// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _WAVE_READER_HPP_cm110729_
#define _WAVE_READER_HPP_cm110729_

#include <iostream>
#include <fstream>
#include <complex>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>

#include "processor.hpp"
#include "processor/IQBuffer.hpp"
#include "network/protocol.hpp"
#include "wave/definitions.hpp"

namespace wave {
  namespace detail { // anonymous 
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
      const double norm(2. / static_cast<double>(i_max));
      return ( ((a&0x80) == 0x80) ? sum-i_max : sum) * norm;
    }    

    class service_wave_iq : public processor::base_iq::service {
    public:
      virtual ~service_wave_iq() {}
      typedef boost::shared_ptr<service_wave_iq> sptr;
      static sptr make(ptime approx_ptime,
                       std::string stream_name,
                       double center_frequency_Hz,
                       const chunk::format& format) {
        return sptr(new service_wave_iq(approx_ptime, stream_name,
                                        center_frequency_Hz, format));
      }
      
      virtual std::string     id()                  const { return "WAVE_000"; }
      virtual ptime           approx_ptime()        const { return approx_ptime_; }
      virtual boost::uint16_t stream_number()       const { return 1; }
      virtual std::string     stream_name()         const { return stream_name_; }      
      virtual boost::uint32_t sample_rate_Hz()      const { return format_.sampleRate(); }
      virtual double          center_frequency_Hz() const { return center_frequency_Hz_; }
      virtual float           offset_ppb()          const { return 0; }
      virtual float           offset_ppb_rms()      const { return 0; }

      virtual void put_result(processor::result_base::sptr rp) {}
      virtual processor::result_base::sptr get_result(std::string name) const { return processor::result_base::sptr();  }

    private:
      service_wave_iq(ptime approx_ptime,
                      std::string stream_name,
                      double center_frequency_Hz,
                      const chunk::format& format)
        : approx_ptime_(approx_ptime)
        , stream_name_(stream_name)
        , center_frequency_Hz_(center_frequency_Hz)
        , format_(format) {}
      
      const ptime         approx_ptime_;
      const std::string   stream_name_;
      const double        center_frequency_Hz_;
      const chunk::format format_;    
    } ;
  } // namespace detail

  template<typename PROCESSOR>
  class reader_iq_base : public processor::base_iq {
  public:
    typedef boost::shared_ptr<reader_iq_base> sptr;

    reader_iq_base(const boost::property_tree::ptree& config)
      : base_iq(config)
      , p_(config)
      , read_riff_(false)
      , read_format_(false)
      , read_data_(false)
      , buffer_length_sec_(config.get<double>("<xmlattr>.buffer_length_sec", 1.0))
      , overlap_(config.get<double>("<xmlattr>.overlap", 0.0))
      , iq_buffer_(0, 0.)
      , sample_number_(0) {}
    virtual ~reader_iq_base() {}

    bool process_file(std::string filename) {
      std::ifstream ifs(filename.c_str());
      start_time_ = boost::posix_time::from_time_t(boost::filesystem::last_write_time(filename));
      return read_chunks(ifs);
    }
    void finish() {
      // insert one last dummy sample to trigger processing of left over data
      iq_buffer_.insert(this, std::complex<double>(0, 0));
    }

    // complex_vector_type samples() const { return iq_buffer_.samples(); }
    
    void procIQ(const_iterator i0,
                const_iterator i1) {
      using namespace boost::posix_time;
      p_.process_iq(get_service(std::distance(i0, i1)), i0, i1);
      start_time_ += time_duration(0, 0, 0,
                                   boost::int64_t(double(std::distance(i0, i1))
                                                  /double(format_.sampleRate())
                                                  *time_duration::ticks_per_second()+0.5));
    }

    void process_iq(service::sptr service,
                    const_iterator i0, 
                    const_iterator i1) { }

  protected:
    const chunk::format& format() const { return format_; }
    ptime start_time() const { return start_time_; }
    void update_start_time(ptime t) { start_time_ = t; }
    boost::int64_t sample_number() const { return sample_number_; }

    virtual service::sptr get_service(size_t number_of_samples) const = 0;

    virtual void reset() {
      read_riff_   = false;
      read_format_ = false;
      read_data_   = false;
    }

    bool read_chunks(std::istream& is) {
      reset();
      riff_      = detail::readT<chunk::riff>(is);
      read_riff_ = true;
      std::cout << riff_ << std::endl;
      if (!riff_.ok()) return false;
      while (is && is.tellg() < int(sizeof(chunk::header)+riff_.size())) {
        const chunk::header h(detail::read_header(is));
        if (h.id() == "fmt ") {
          format_      = detail::readT<chunk::format>(is);
          read_format_ = true;
          std::cout << format_ << std::endl;
          iq_buffer_.update(size_t(format_.sampleRate()*buffer_length_sec_+0.5), overlap_);
        } else if (h.id() == "data") {
          data_      = detail::readT<chunk::data>(is);
          read_data_ = true;
          std::cout << data_ << std::endl;
          // process data here:
          // is.seekg(h.size(), std::ios_base::cur);
          for (size_t i=0; i<h.size(); i+=format_.bytesPerSample()) {
            const double xi(detail::read_real_sample(is, format_.bitsPerSample())); // real
            const double xq(detail::read_real_sample(is, format_.bitsPerSample())); // imag
            iq_buffer_.insert(this, std::complex<double>(xi, xq));
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
    PROCESSOR p_;
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
    reader_iq(const boost::property_tree::ptree& config)
      : reader_iq_base<PROCESSOR>(config)
      , center_freq_hz_(config.get<double>("<xmlattr>.center_freq_Hz", 0.0)) {
      update_start_time(boost::posix_time::microsec_clock::universal_time());
    }    
    virtual ~reader_iq() {}

    typedef typename reader_iq_base<PROCESSOR>::service service;
    using reader_iq_base<PROCESSOR>::get_service;
    using reader_iq_base<PROCESSOR>::format;
    using reader_iq_base<PROCESSOR>::start_time;
    using reader_iq_base<PROCESSOR>::update_start_time;

    virtual typename service::sptr get_service(size_t number_of_samples) const {
      return detail::service_wave_iq::make(start_time(),
                                           "name",
                                           center_freq_hz_,
                                           format());                                           
    }

  private:
    double center_freq_hz_;
  } ;
#if 0  
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
      rcvr_       = detail::readT<chunk::rcvr>(is);
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
#endif
} // namespace wave
#endif // _WAVE_READER_HPP_cm110729_
