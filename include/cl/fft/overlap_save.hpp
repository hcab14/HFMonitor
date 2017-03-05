// -*- C++ -*-

#ifndef _CL_OVERLAP_SAVE_HPP_cm170305_
#define _CL_OVERLAP_SAVE_HPP_cm170305_

#include <fstream>
#include <boost/format.hpp>

#include "filter/fir/overlap_save_base.hpp"
#include "cl/fft.hpp"

namespace cl {
  namespace fft {
    class overlap_save : public filter::fir::overlap_save_base {
    public:
      typedef array<complex_type>::vector_type complex_vector_type;
      typedef FFT::FFTWTransform<float> small_fft_type;

      typedef boost::shared_ptr<overlap_save> sptr;

    private:
      // class for holding filter coefficients and the fft
      class filt {
      public:
        typedef typename boost::shared_ptr<filt> sptr;

        template<typename U>
        filt(::size_t l,
             ::size_t p,
             const typename std::vector<U>& b,
             double shift, // normalized frequency
             ::size_t d,    // downsampling factor
	     cl::Context& ctx,
	     cl::Program& program,
	     cl::CommandQueue& queue,
	     cl::Buffer&  in_buffer)
          : l_(l)
          , p_(p)
          , d_(d)
          , n_(l_+p_-1)
          , shift_(::size_t((1.+shift)*n_+0.5) % n_)
	  , fft_ (n_,     1, FFTW_ESTIMATE)
	    //	  , ifft_(n_/d_, -1, FFTW_ESTIMATE)
	    //    , h_(n_, 0)
	  , in_ (ctx, n_/d_)
	  , out_(ctx, n_/d_)
	  , h_  (ctx, n_)
	  , kernel_(program, "convolve")
	{
          assert(l_+p_ > 0);
          assert(((p_-1)%d_) == 0);
          assert(b.size() < n());
          assert((n_%d_) == 0);
          assert((l_%d_) == 0);
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
	  kernel_.setArg(0, in_.get_device_buffer());
	  kernel_.setArg(1, in_buffer);
	  kernel_.setArg(2, h_.get_device_buffer());
	  kernel_.setArg(3, cl_int(n_));
	  kernel_.setArg(4, cl_int(d_));
	  kernel_.setArg(5, cl_int(shift_));
	  kernel_.setArg(6, cl_float(1.0f/n_)); // norm

	  const float scale = 1.0f;
	  ::size_t clLengths[1] = { n_/d_ };
	  ASSERT_THROW_CL(clfftCreateDefaultPlan(&plan_handle_, ctx(), CLFFT_1D, clLengths));
	  ASSERT_THROW_CL(clfftSetPlanPrecision ( plan_handle_, CLFFT_SINGLE));
	  ASSERT_THROW_CL(clfftSetPlanScale     ( plan_handle_, CLFFT_BACKWARD, scale));
	  ASSERT_THROW_CL(clfftSetLayout        ( plan_handle_, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED));
	  ASSERT_THROW_CL(clfftSetResultLocation( plan_handle_, CLFFT_OUTOFPLACE));
	  ASSERT_THROW_CL(clfftBakePlan         ( plan_handle_, 1, &queue(), NULL, NULL));
	}

        ~filt() {
	  clfftDestroyPlan(&plan_handle_);
	}

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
	complex_vector_type::const_iterator begin()  const { return out_.begin(); }
	complex_vector_type::const_iterator end()    const { return out_.end(); }
        const complex_vector_type&          result() const { return out_.get(); }

        // performs inverse FFT of (shifted) input and downsampling
	cl::Event transform(cl::CommandQueue& q, const std::vector<cl::Event>& waitFor) {
	  cl::Event event1;
	  const auto dev = q.getInfo<CL_QUEUE_DEVICE>();
	  const int max_work_group_size = kernel_.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(dev);

	  const int nd = n()/d();
	  int work_group_size = max_work_group_size;
	  for (; work_group_size && (nd%work_group_size); --work_group_size)
	    ;

	  q.enqueueNDRangeKernel(kernel_,
				 cl::NullRange,                // offset
				 cl::NDRange(nd),              // global
				 cl::NDRange(work_group_size), // local
				 &waitFor,
				 &event1);

	  std::vector<cl::Event> event2(1, cl::Event());
	  ASSERT_THROW_CL(clfftEnqueueTransform(plan_handle_, CLFFT_BACKWARD, 1, &q(),
						1, &event1(), &event2[0](),
						&in_.get_device_buffer()(), &out_.get_device_buffer()(), NULL));
	  cl::Event event3;
	  out_.device_to_host(q, &event2, &event3);

	  return event3;
        }

      protected:
      private:
        const ::size_t      l_;
        const ::size_t      p_;
        const ::size_t      d_;
        const ::size_t      n_;
        ::size_t            shift_;
        small_fft_type      fft_;
	array<complex_type> in_;
	array<complex_type> out_;
	array<complex_type> h_;
	cl::Kernel          kernel_;
	clfftPlanHandle     plan_handle_;
      } ;

    public:
      overlap_save(::size_t l, // Number of new input samples consumed per data block
                   ::size_t p, // Length of h(n)
		   cl::Context& ctx,
		   cl::CommandQueue& queue)
        : filter::fir::overlap_save_base()
	, l_(l)
	, p_(p)
        , last_id_(0)
	, in_ (ctx, l+p-1)
	, out_(ctx, l+p-1)
	, queue_(queue)
	, prog_source_(read_kernel_source("include/cl/fft/kernel.cl"))
	, program_(ctx, cl::Program::Sources(1, std::make_pair(prog_source_.c_str(),
							       prog_source_.length()+1))) {
        for (::size_t i(0), iend(l+p-1); i<iend; ++i)
          in_[i] = out_[i] = 0;

	const float scale = 1.0f;
	::size_t clLengths[1] = { l+p-1 };
	ASSERT_THROW_CL(clfftCreateDefaultPlan(&plan_handle_, ctx(), CLFFT_1D, clLengths));
	ASSERT_THROW_CL(clfftSetPlanPrecision ( plan_handle_, CLFFT_SINGLE));
	ASSERT_THROW_CL(clfftSetPlanScale     ( plan_handle_, CLFFT_FORWARD, scale));
	ASSERT_THROW_CL(clfftSetLayout        ( plan_handle_, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED));
	ASSERT_THROW_CL(clfftSetResultLocation( plan_handle_, CLFFT_OUTOFPLACE));
	ASSERT_THROW_CL(clfftBakePlan         ( plan_handle_, 1, &queue(), NULL, NULL));

	auto const& devices = ctx.getInfo<CL_CONTEXT_DEVICES>();
	program_.build(devices, "");
      }

      virtual ~overlap_save() {
	clfftDestroyPlan(&plan_handle_);
      }

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
        if (b.size() != p_)
          throw std::runtime_error("overlap_save::update_filter_coeff b.size() != p_");
	cl::Context ctx = queue_.getInfo<CL_QUEUE_CONTEXT>();
        typename filt::sptr fp(new filt(l_, p_, b, offset, decim, ctx, program_, queue_, out_.get_device_buffer()));
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
          in_[p_+i-1] = *i0++;

	cl::Event event0;
	in_.host_to_device(queue_, NULL, &event0);

	std::vector<cl::Event> events(1, cl::Event());
	ASSERT_THROW_CL(clfftEnqueueTransform(plan_handle_, CLFFT_FORWARD, 1, &queue_(), 1, &event0(),
					      &events[0](),
					      &in_.get_device_buffer()(), &out_.get_device_buffer()(), NULL));

        // for each filter
	std::vector<cl::Event> events_wait;
        for (auto filter : filters_)
          events_wait.push_back(filter.second->transform(queue_, events));

	cl::Event::waitForEvents(events_wait);
	cl::finish();

        // save old samples
        for (::size_t i(0), iend(p_-1); i<iend; ++i)
          in_[i] = in_[l_+i];
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

      array<complex_type> in_;
      array<complex_type> out_;
      clfftPlanHandle     plan_handle_;
      cl::CommandQueue    queue_;
      std::string         prog_source_;
      cl::Program         program_;
    } ;
  } // namespace fft
} // namespace cl


#endif // _CL_OVERLAP_SAVE_HPP_cm170305_
