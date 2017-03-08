// -*- C++ -*-

#ifndef _CL_OVERLAP_SAVE_HPP_cm170305_
#define _CL_OVERLAP_SAVE_HPP_cm170305_

#include <fstream>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "processor.hpp"
#include "cl/fft.hpp"

namespace cl {
  namespace fft {
    class overlap_save {
    public:
      typedef boost::shared_ptr<overlap_save> sptr;
      typedef processor::base_iq::complex_type complex_type;
      typedef array<complex_type>::vector_type complex_vector_type;
      typedef FFT::FFTWTransform<float> small_fft_type;

    private:
      // class for holding filter coefficients and the fft
      class filt {
      public:
        typedef typename boost::shared_ptr<filt> sptr;

        template<typename U>
        filt(::size_t l,
             ::size_t p,
             const typename std::vector<U>& b,
             double shift,  // normalized frequency
             ::size_t d,    // downsampling (decimation) factor
	     cl::Program& program,
	     cl::CommandQueue& queue,
	     cl::Buffer&  in_buffer)
          : l_(l)
          , p_(p)
          , d_(d)
          , n_(l_+p_-1)
          , shift_(::size_t((1.+shift)*n_+0.5) % n_)
	  , fft_ (n_,     1, FFTW_ESTIMATE)
	  , clfft_(n_/d_, CLFFT_BACKWARD, queue)
	  , h_  (queue, n_)
	  , kernel_(program, "convolve")
	{
	  verify_arguments();
          ASSERT_THROW(b.size() < n());

          complex_vector_type in(n(), 0);
          std::copy(b.begin(), b.end(), in.begin());
          fft_.transformVector(in, FFT::WindowFunction::Rectangular<float>(b.size()));
          for (::size_t i(0), iend(n()); i<iend; ++i) {
            h_[i] = fft_.out(i);
            fft_.in(i) = 0;
          }
	  h_.host_to_device(queue);

          for (::size_t i(0), nd(n()/d_); i<nd; ++i)
            in[i] = 0;
          shift_ = (shift_/::size_t(v()+.5))*::size_t(v()+0.5);

	  // set up kernel
	  kernel_.setArg(0, clfft_.in().get_device_buffer());
	  kernel_.setArg(1, in_buffer);
	  kernel_.setArg(2, h_.get_device_buffer());
	  kernel_.setArg(3, cl_int(n_));
	  kernel_.setArg(4, cl_int(d_));
	  kernel_.setArg(5, cl_int(shift_));
	  kernel_.setArg(6, cl_float(1.0f/n_)); // norm
	}

        ~filt() {}

        ::size_t l() const { return l_; }                  // Number of new input samples consumed per data block
        ::size_t p() const { return p_; }                  // Length of h(n)
        ::size_t d() const { return d_; }                  // downsampling factor
        ::size_t n() const { return n_; }                  // FFT size
        ::size_t shift() const { return shift_; }          // frequency shift
        double v() const { return double(n())/(p_-1.); }   // Overlap factor

        double offset() const {
          return (shift() > n()/2
		  ? double(int(shift())-int(n()))/n()
		  : double(shift())/n());
        }
	complex_vector_type::const_iterator begin()  const { return clfft_.out().begin(); }
	complex_vector_type::const_iterator end()    const { return clfft_.out().begin()+l()/d(); }

        // performs inverse FFT of (shifted) input and downsampling
	cl::Event transform(cl::CommandQueue& q, const std::vector<cl::Event>& waitFor) {
	  const auto dev = q.getInfo<CL_QUEUE_DEVICE>();
	  const int max_work_group_size = kernel_.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(dev);

	  const int nd = n()/d();
	  int work_group_size = max_work_group_size;
	  for (; work_group_size && (nd%work_group_size); --work_group_size)
	    ;

	  std::vector<cl::Event> event1(1, cl::Event());;
	  q.enqueueNDRangeKernel(kernel_,
				 cl::NullRange,                // offset
				 cl::NDRange(nd),              // global
				 cl::NDRange(work_group_size), // local
				 &waitFor,
				 &event1[0]);

	  std::vector<cl::Event> event2(1, clfft_.enqueue_transform(event1));

	  cl::Event event3;
	  clfft_.device_to_host(&event2, &event3);

	  return event3;
        }

      protected:
	void verify_arguments() const {
          ASSERT_THROW(l_+p_ > 0);
          ASSERT_THROW(((p_-1)%d_) == 0);
          ASSERT_THROW((n_%d_) == 0);
          ASSERT_THROW((l_%d_) == 0);
	}
      private:
        const ::size_t      l_;
        const ::size_t      p_;
        const ::size_t      d_;
        const ::size_t      n_;
        ::size_t            shift_;
        small_fft_type      fft_;
	cl::fft::clfft      clfft_;
	array<complex_type> h_;
	cl::Kernel          kernel_;
      } ;

    public:
      overlap_save(::size_t l, // Number of new input samples consumed per data block
                   ::size_t p, // Length of h(n)
		   cl::CommandQueue& queue)
        : l_(l)
	, p_(p)
        , last_id_(0)
	, clfft_(l+p-1, CLFFT_FORWARD, queue)
	, queue_(queue)
	, prog_source_(read_kernel_source("include/cl/fft/kernel.cl"))
	, program_(queue.getInfo<CL_QUEUE_CONTEXT>(),
		   cl::Program::Sources(1, std::make_pair(prog_source_.c_str(),
							  prog_source_.length()+1))) {
	auto const& ctx     = queue.getInfo<CL_QUEUE_CONTEXT>();
	auto const& devices = ctx.getInfo<CL_CONTEXT_DEVICES>();
	program_.build(devices, "");
      }

      virtual ~overlap_save() {}

      virtual ::size_t l() const { return l_; }
      virtual ::size_t p() const { return p_; }

      typedef std::map<::size_t, typename filt::sptr> filter_map;

      typename filt::sptr get_filter(::size_t index) const {
	//        return filters_[index];
	auto const& i = filters_.find(index);
	if (i != filters_.end())
	  return i->second;
	else
	  throw 1;
      }

      // add one filter
      //  * returns a pair of handle (::size_t) and the (rounded) mid-frequency of the filter
      virtual std::pair<::size_t, double> add_filter(const std::vector<float>& b,
						     double_t offset,
						     ::size_t decim) {
        ASSERT_THROW(b.size() == p_);
        typename filt::sptr fp(new filt(l_, p_, b, offset, decim, program_, queue_, clfft_.out().get_device_buffer()));
        filters_.insert(std::make_pair(last_id_, fp));
        return std::make_pair(last_id_++, fp->offset());
      }

      virtual processor::base_iq::const_iterator begin(::size_t idx) const {
	return processor::base_iq::const_iterator(get_filter(idx)->begin());
      }
      virtual processor::base_iq::const_iterator end  (::size_t idx) const {
	return processor::base_iq::const_iterator(get_filter(idx)->end());
      }

      // void proc(const complex_vector_type& in) { proc(in.begin(), in.end()); }

      virtual void proc(processor::base_iq::const_iterator i0,
			processor::base_iq::const_iterator i1) {
        if (std::distance(i0, i1) != int(l_))
          throw std::runtime_error(str(boost::format("overlap_save::proc in.size() != l_ : %d != %d")
                                       % std::distance(i0, i1)
                                       % int(l_)));

        // copy input data
        for (::size_t i(0); i<l_; ++i)
          clfft_.in(p_+i-1) = *i0++;

	std::vector<cl::Event> event0(1, cl::Event());
	clfft_.host_to_device(NULL, &event0[0]);

	std::vector<cl::Event> event1(1, clfft_.enqueue_transform(event0));

        // for each filter
	std::vector<cl::Event> events_wait;
        for (auto filter : filters_)
          events_wait.push_back(filter.second->transform(queue_, event1));

	cl::Event::waitForEvents(events_wait);

        // save old samples
        for (::size_t i(0), iend(p_-1); i<iend; ++i)
          clfft_.in(i) = clfft_.in(l_+i);
      }

    private:
      static std::string read_kernel_source(std::string filename) {
	std::ifstream file(filename);
	return std::string(std::istreambuf_iterator<char>(file),
			   (std::istreambuf_iterator<char>()));
      }

      const ::size_t l_;
      const ::size_t p_;
      ::size_t last_id_;

      filter_map filters_;

      cl::fft::clfft   clfft_;
      cl::CommandQueue queue_;
      std::string      prog_source_;
      cl::Program      program_;
    } ;

    class overlap_save_setup {
    public:
      typedef overlap_save::sptr sptr;
      overlap_save_setup()
	: _queue(make_queue())
	, _ctx(_queue.getInfo<CL_QUEUE_CONTEXT>()) {}

      sptr make_overlap_save(::size_t l, ::size_t p) {
	return boost::make_shared<overlap_save>(l, p, _queue);
      }

    protected:
      static cl::CommandQueue make_queue() {
	std::vector<cl::Platform> platforms;
	cl::Platform::get(&platforms);
	ASSERT_THROW(!platforms.empty());

	auto const& default_platform = platforms[0];
	std::cout << "Using platform: "<< default_platform.getInfo<CL_PLATFORM_NAME>() << " #platforms=" << platforms.size() << "\n";

	cl_context_properties properties[] =
	  { CL_CONTEXT_PLATFORM, (cl_context_properties)(platforms[0])(), 0};
	cl::Context ctx(CL_DEVICE_TYPE_GPU, properties);

	std::vector<cl::Device> devices = ctx.getInfo<CL_CONTEXT_DEVICES>();
	ASSERT_THROW(!devices.empty());

	auto const& default_device = devices[0];
	return cl::CommandQueue(ctx, default_device);
      }

    private:
      cl::CommandQueue _queue;
      cl::Context      _ctx;
    } ;

  } // namespace fft
} // namespace cl


#endif // _CL_OVERLAP_SAVE_HPP_cm170305_
