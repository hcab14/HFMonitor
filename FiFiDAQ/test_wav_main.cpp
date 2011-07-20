// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <fstream>
#include <complex>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>

#include "WAVE.hpp"
#include "IQBuffer.hpp"
#include "protocol.hpp"

namespace processor {

  class specific_param_base {
  public:
    typedef boost::shared_ptr<specific_param_base> sptr;
    virtual ~specific_param_base() {}

    virtual std::string to_string() const {
      return "specific_param_base"; 
    }
  } ;

  class perseus_param : public specific_param_base {
  public:
    typedef boost::shared_ptr<perseus_param> sptr;
    perseus_param(protocol::perseus::header header)
      : header_(header) {}
    virtual ~perseus_param() {}

    // TODO
    virtual std::string to_string() const {
      return "perseus_param_base";
    }
  private:
    const protocol::perseus::header header_;
  } ;

  class service_base {
  public:
    typedef boost::posix_time::ptime ptime;
    typedef boost::shared_ptr<service_base> sptr;

    service_base(std::string source_name)
      : source_name_(source_name) {}
    virtual ~service_base() {}

    // name of iq data source
    virtual std::string source_name() const { return source_name_; }

    virtual double center_freq_hz() const = 0;
    virtual double sample_rate_hz() const = 0;

    // approx. time at start of samples
    virtual ptime approx_ptime() const = 0;

    // param specific to the source
    virtual specific_param_base::sptr specific_param() const = 0;

    // return a service object with samplerate changed by factor num/denum
    virtual sptr resample(size_t num, size_t denum) const = 0;

    // return a service object with a shifted center frequency
    virtual sptr freq_shift(double freq_shift_hz) const = 0;

  protected:
  private:
    const std::string source_name_;
  } ;

  class service_generic_iq : public service_base {
  public:
    typedef boost::shared_ptr<service_generic_iq> sptr;

    service_generic_iq(double center_freq_hz,
                       double sample_rate_hz,
                       ptime  approx_ptime)
      : service_base("generic I/Q")
      , center_freq_hz_(center_freq_hz)
      , sample_rate_hz_(sample_rate_hz)
      , approx_ptime_(approx_ptime) {}

    virtual ~service_generic_iq() {}

    virtual double center_freq_hz() const { return center_freq_hz_; }
    virtual double sample_rate_hz() const { return sample_rate_hz_; }

    // approx. time at start of samples
    virtual ptime approx_ptime() const { return approx_ptime_; }

    // param specific to the source
    virtual specific_param_base::sptr specific_param() const {
      return specific_param_base::sptr(new specific_param_base);
    }

    // return a service object with samplerate changed by factor num/denum
    virtual service_base::sptr resample(size_t num, size_t denum) const {
      return sptr(new service_generic_iq(center_freq_hz_,
                                         sample_rate_hz_*num/denum,
                                         approx_ptime_));
    }

    // return a service object with a shifted center frequency
    virtual service_base::sptr freq_shift(double freq_shift_hz) const {
      return sptr(new service_generic_iq(center_freq_hz_+freq_shift_hz,
                                         sample_rate_hz_,
                                         approx_ptime_));
    }

  protected:
  private:
    double center_freq_hz_;
    double sample_rate_hz_;
    ptime  approx_ptime_;
  } ;

  class service_perseus : public service_base {
  public:
    typedef boost::posix_time::ptime ptime;
    typedef boost::shared_ptr<service_perseus> sptr;
    
    service_perseus(const protocol::perseus::header& header)
      : service_base("service_perseus")
      , header_(header) {}
    virtual ~service_perseus() {}

    virtual double center_freq_hz() const { return header_.ddcCenterFrequency(); }
    virtual double sample_rate_hz() const { return header_.sampleRate(); }

    // approx. time at start of samples
    virtual ptime approx_ptime() const { return header_.approxPTime(); }

    // param specific to the source
    virtual specific_param_base::sptr specific_param() const {
      return perseus_param::sptr(new perseus_param(header_));
    }

    // return a service object with samplerate changed by factor num/denum
    virtual service_base::sptr resample(size_t num, size_t denum) const {
      return sptr(new service_perseus(header_.resample(num, denum)));
    }  
    // return a service object with a shifted center frequency
    virtual service_base::sptr freq_shift(double freq_shift_hz) const {
      return sptr(new service_perseus(header_.freq_shift(freq_shift_hz)));
    }

  protected:
  private:
    const protocol::perseus::header header_;
  } ;  
} // namespace processor

namespace wave {
  namespace { // anonymous 
    template<typename T>
    T readT(std::istream& is) {
      T data;
      is.read((char *)(&data), sizeof(T));
      if (!is) throw std::runtime_error("read failed");
      return data;
    }
    chunk::header read_header(std::istream& is) {
      const std::streampos pos(is.tellg());
      const chunk::header header(readT<chunk::header>(is));
      is.seekg(pos);
      return header;
    }
    double read_real_sample(std::istream& is,
                            size_t bits_per_sample) {
      boost::uint8_t a(0);
      boost::int32_t sum(0);
      for (size_t u(0); u<bits_per_sample; u+=8) 
        sum |= ((a=readT<boost::uint8_t>(is)) << u);
      const boost::int32_t i_max(1L << bits_per_sample);
      const double norm(1. / static_cast<double>(i_max));
      return ( ((a&0x80) == 0x80) ? sum-i_max : sum) * norm;
    }    
  } // namespace anonymous
  
  template<typename PROCESSOR>
  class reader_iq_base : public boost::noncopyable {
  public:
    typedef std::complex<double> complex_type;
    typedef std::vector<complex_type> complex_vector_type;
    typedef boost::shared_ptr<reader_iq_base> sptr;
    typedef boost::posix_time::ptime ptime;

    reader_iq_base(PROCESSOR& p,
                   double buffer_length_sec,
                   double overlap)
      : p_(p)
      , read_riff_(false)
      , read_format_(false)
      , read_data_(false)
      , buffer_length_sec_(buffer_length_sec)
      , overlap_(overlap)
      , iq_buffer_(0, 0.)
      , sample_number_(0) {}
    virtual ~reader_iq_base() {}

    bool process_file(std::string filename) {
      std::ifstream ifs(filename.c_str());
      start_time_ = boost::posix_time::from_time_t(boost::filesystem::last_write_time(filename));
      return read_chunks(ifs);
    }

    complex_vector_type samples() const { return iq_buffer_.samples(); }

    // called from IQBuffer::insert
    void process_iq(IQBuffer::Samples::const_iterator i0, 
                    IQBuffer::Samples::const_iterator i1) {
      using namespace boost::posix_time;
      p_.process_iq(get_service(std::distance(i0, i1)), i0, i1);
      start_time_ += time_duration(0, 0, 0,
                                    double(std::distance(i0, i1))
                                   /double(format_.sampleRate())
                                   *time_duration::ticks_per_second());
    }

  protected:
    const chunk::format& format() const { return format_; }
    ptime start_time() const { return start_time_; }
    void update_start_time(ptime t) { start_time_ = t; }
    boost::int64_t sample_number() const { return sample_number_; }

    virtual processor::service_base::sptr get_service(size_t number_of_samples) const = 0;

    virtual void reset() {
      read_riff_   = false;
      read_format_ = false;
      read_data_   = false;
    }

    bool read_chunks(std::istream& is) {
      reset();
      riff_      = readT<chunk::riff>(is);
      read_riff_ = true;
      std::cout << riff_ << std::endl;
      if (!riff_.ok()) return false;
      while (is && is.tellg() < sizeof(chunk::header)+riff_.size()) {
        const chunk::header h(read_header(is));
        if (h.id() == "fmt ") {
          format_      = readT<chunk::format>(is);
          read_format_ = true;
          std::cout << format_ << std::endl;
          iq_buffer_.update(format_.sampleRate() * buffer_length_sec_, overlap_);
        } else if (h.id() == "data") {
          data_      = readT<chunk::data>(is);
          read_data_ = true;
          std::cout << data_ << std::endl;
          // process data here:
          // is.seekg(h.size(), std::ios_base::cur);
          for (size_t i=0; i<h.size(); i+=format_.bytesPerSample()) {
            const double xi(read_real_sample(is, format_.bitsPerSample()));
            const double xq(read_real_sample(is, format_.bitsPerSample()));
            iq_buffer_.insert(this, complex_type(xi, xq));
            ++sample_number_;
          }
        } else if (read_chunk(h, is)) {
          // further types of chunks can be read in a derived class
        } else {
          std::cout << "unknown subchunk '"<< h.id()
                    << "' skipping "<< sizeof(chunk::header)+h.size() << " bytes"
                    << std::endl;
          is.seekg(sizeof(chunk::header)+h.size(), std::ios_base::cur);
        }
      }
      return chunks_ok();
    }
    // to be overwritten in derived classes
    virtual bool read_chunk(const chunk::header& h,
                            std::istream& is) { 
      return false;
    }

    virtual bool chunks_ok() const { 
      return (   read_riff_   && riff_.ok()
              && read_format_ && format_.ok()
              && read_data_   && data_.ok());
    }

  private:
    PROCESSOR& p_;
    bool read_riff_;
    bool read_format_;
    bool read_data_;

    chunk::riff   riff_;
    chunk::format format_;
    chunk::data   data_;

    double buffer_length_sec_;
    double overlap_;
    IQBuffer iq_buffer_;

    ptime start_time_; // current start time
    boost::int64_t sample_number_; // current sample number
  } ;

  template<typename PROCESSOR>
  class reader_iq : public reader_iq_base<PROCESSOR> {
  public:
    reader_iq(PROCESSOR& p,
              double center_freq_hz,
              boost::posix_time::ptime start_time,
              double buffer_length_sec, 
              double overlap)
      : reader_iq_base<PROCESSOR>(p, buffer_length_sec, overlap)
      , center_freq_hz_(center_freq_hz) {
      update_start_time(start_time);
    }
    
    virtual ~reader_iq() {}

    using reader_iq_base<PROCESSOR>::get_service;
    using reader_iq_base<PROCESSOR>::format;
    using reader_iq_base<PROCESSOR>::start_time;
    using reader_iq_base<PROCESSOR>::update_start_time;
    
    virtual processor::service_base::sptr get_service(size_t ) const {
      return processor::service_generic_iq::sptr
        (new processor::service_generic_iq(format().sampleRate(),
                                           center_freq_hz_,
                                           start_time()));
    }

  private:
    double center_freq_hz_;
  } ;
  
  template<typename PROCESSOR>
  class reader_perseus : public reader_iq_base<PROCESSOR> {
  public:
    typedef boost::posix_time::ptime ptime;

    reader_perseus(PROCESSOR& p,
                   double buffer_length_sec, 
                   double overlap)
      : reader_iq_base<PROCESSOR>(p, buffer_length_sec, overlap)
      , read_rcvr_(false) {}
    
    virtual ~reader_perseus() {}

    using reader_iq_base<PROCESSOR>::reset;
    using reader_iq_base<PROCESSOR>::read_chunk;
    using reader_iq_base<PROCESSOR>::chunks_ok;
    using reader_iq_base<PROCESSOR>::get_service;
    using reader_iq_base<PROCESSOR>::format;
    using reader_iq_base<PROCESSOR>::start_time;
    using reader_iq_base<PROCESSOR>::update_start_time;
    using reader_iq_base<PROCESSOR>::sample_number;

    virtual processor::service_base::sptr get_service(size_t number_of_samples) const {
      const protocol::perseus::header h(sample_number(),
                                        format().sampleRate(),
                                        rcvr_.nCenterFrequencyHz(),
                                        number_of_samples,
                                        rcvr().nSamplingRateIdx(),
                                        rcvr().wAttenId(),
                                        rcvr().bAdcPresel(),
                                        rcvr().bAdcPreamp(),
                                        rcvr().bAdcDither());                             
      return processor::service_perseus::sptr(new processor::service_perseus(h));
    }

  protected:
    virtual void reset() {
      reader_iq_base<PROCESSOR>::reset();
      read_rcvr_ = false;
    }
    virtual bool read_chunk(const chunk::header& h,
                            std::istream& is) { 
      if (h.id() != "rcvr")
        return false;
      rcvr_       = readT<chunk::rcvr>(is);
      read_rcvr_  = true;
      update_start_time(rcvr_.ptimeStart());
      return rcvr_.ok();
    }
    virtual bool chunks_ok() const {
      return reader_iq_base<PROCESSOR>::chunks_ok() && read_rcvr_ && rcvr_.ok();
    }
    const chunk::rcvr rcvr() const { return rcvr_; }
  private:
    bool read_rcvr_;
    chunk::rcvr rcvr_;
  } ;
} // namespace wave

class test_proc {
public:
  void process_iq(processor::service_base::sptr service,
                  IQBuffer::Samples::const_iterator i0, 
                  IQBuffer::Samples::const_iterator i1) {
    std::cout << "test_proc::process_iq " << service->approx_ptime() << std::endl;
  }
protected:
private:
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", argv[0]);
  test_proc p;
  wave::reader_iq<test_proc> wr(p, 0.0, boost::posix_time::ptime(), 1.0, 0.0);
  wr.process_file("wav/iq_300kHz.wav");
  std::cout << "samples  " << wr.samples().size() << std::endl;

  wave::reader_perseus<test_proc> wrp(p, 1.0, 0.0);
  return 0;
}
