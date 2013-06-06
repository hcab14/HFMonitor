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
#ifndef _WAVE_WRITER_HPP_cm121217_
#define _WAVE_WRITER_HPP_cm121217_

#include <iostream>
#include <fstream>
#include <complex>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>

#include "gen_filename.hpp"
#include "processor.hpp"
#include "network/protocol.hpp"
#include "wave/definitions.hpp"

namespace wave {
  namespace detail {
    template<typename T>
    std::ostream& writeT(std::ostream& os, const T& data) {
      return os.write((char *)(&data), sizeof(T));
    }
    void write_real_sample(std::ostream& os, boost::uint16_t bitsPerSample, double s) {
      // we assume abs(s)<=1
      boost::uint32_t si= boost::uint32_t(std::abs(s) * (1L << (bitsPerSample-1)));
      if (s < 0.) si= 1L + (0xFFFFFFFFL ^ si);
      for (size_t u(0); u<bitsPerSample; u+=8) {
        const boost::uint8_t c((si >> u) & 0xFF);
        writeT(os, c);
      }
    }

    class wave_header {
    public:
      wave_header(boost::uint32_t sampleRate,
                  boost::uint16_t bitsPerSample,
                  boost::uint16_t numChannels)
        : format_(sampleRate,
                  bitsPerSample,
                  numChannels) {}

      wave_header& change_sample_rate(boost::uint32_t new_sample_rate) {
        format_.changeSampleRate(new_sample_rate);
        data_.size() = 0;
        return *this;
      }
      wave_header& add_samples(size_t number_of_samples) {
        data_.size() +=  number_of_samples * format_.numChannels() * format_.bitsPerSample()/8;
        riff_.size() =  4 + (8 + format_.size()) + (8 + data_.size());
        return *this;
      }

      const chunk::riff&   riff()   const { return riff_; }
      const chunk::format& format() const { return format_; }
      const chunk::data&   data()   const { return data_; }
    private:
      chunk::riff   riff_;
      chunk::format format_;
      chunk::data   data_;
    } ;
  } // namespace detail

  class writer_iq : public processor::base_iq, public gen_filename {
  public:
    typedef boost::shared_ptr<writer_iq> sptr;

    writer_iq(const boost::property_tree::ptree& config)
      : base_iq(config)
      , base_path_(config.get<std::string>("<xmlattr>.filePath"))
      , file_period_(gen_filename::str2period(config.get<std::string>("<xmlattr>.filePeriod")))
      , pos_(0)
      , header_(0, config.get<int>("<xmlattr>.bitsPerSample", 16), 2) 
        // set correct sampling frequency later, number of channels is fixed to 2
    {
      std::cout << "writer_iq" << std::endl;
    }

    virtual ~writer_iq() {}
    
    // gen_filename methods
    virtual file_period filePeriod()    const { return file_period_; }
    virtual std::string fileExtension() const { return "wav"; }

    virtual void process_iq(service::sptr service,
                            const_iterator i0, 
                            const_iterator i1) {
      std::cout << "writer_iq::process_iq " << service->approx_ptime() << " "
                << pos_ << " " 
                << std::distance(i0, i1) << std::endl;
      const boost::filesystem::path
        filepath(gen_file_path(base_path_, service->stream_name(), service->approx_ptime()));

      if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
        std::cerr << "file '" << filepath << "' exists and will be overwritten" << std::endl;
        boost::filesystem::remove(filepath);
      }
      if (not boost::filesystem::exists(filepath)) {
        std::cerr << "new file " << filepath << " samplerate= " 
                  << service->sample_rate_Hz() << std::endl;
        std::ofstream ofs(filepath.c_str(), std::ios::binary);        
        detail::writeT(ofs, header_.change_sample_rate(service->sample_rate_Hz()));
        pos_ = ofs.tellp();
        std::cerr << "writing header to file '" << filepath << "' exists " << std::endl;
      }
      // write data
      // TBD
      std::ofstream ofs(filepath.c_str(), std::ios::in | std::ios::out | std::ios::binary);
      ofs.seekp(pos_);
      for (processor::base_iq::const_iterator i(i0); i!=i1; ++i) {
        detail::write_real_sample(ofs, header_.format().bitsPerSample(), i->real()); // i
        detail::write_real_sample(ofs, header_.format().bitsPerSample(), i->imag()); // q
      }
      pos_ = ofs.tellp();      
      detail::writeT(ofs.seekp(0), header_.add_samples(std::distance(i0, i1)));
    }

  protected:
  private:
    std::string    base_path_;
    gen_filename::file_period file_period_;
    std::streampos pos_;    
    detail::wave_header header_;
  } ;
} // namespace wave

#endif // _WAVE_WRITER_HPP_cm121217_
