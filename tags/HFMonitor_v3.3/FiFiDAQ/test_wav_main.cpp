// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#include <iostream>
#include "wave_reader.hpp"

namespace wave {
  namespace { // anonymous
    template<typename T>
    std::ostream& writeT(std::ostream& os, const T& data) {
      return os.write((char *)(&data), sizeof(T));
    }
    void write_real_sample(std::ostream& os, boost::uint16_t bitsPerSample, double s) {
      // we assume abs(s)<=1
      boost::uint32_t si= boost::uint32_t(std::abs(s) * (1L << (bitsPerSample-1)));
      if (s < 0.) si= 1L + (0xFFFFFFFFL ^ si);
      for (size_t u=0; u<bitsPerSample; u+=8) {
        const boost::uint8_t c= (si >> u) & 0xFF;
        writeT(os, c);
      }
    }

    class wave_header {
    public:
      wave_header(boost::uint32_t sampleRate,
                  boost::uint16_t bitsPerSample,
                  boost::uint16_t numChannels)
        : format_(sampleRate, bitsPerSample, numChannels) {}

      const wave_header& change_sample_rate(boost::uint32_t new_sample_rate) {
        format_.changeSampleRate(new_sample_rate);
        return *this;
      }
      const wave_header& add_samples(size_t number_of_samples) {
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
  } // namespace anonymous

  class writer_iq {
  public:
    writer_iq(std::string filename,
              boost::uint32_t sampleRate,
              boost::uint16_t bitsPerSample,
              boost::uint16_t numChannels)
      : filename_(filename)
      , pos_(0)
      , header_(sampleRate, bitsPerSample, numChannels) {}
    
    void process_iq(processor::service_base::sptr service,
                    IQBuffer::Samples::const_iterator i0, 
                    IQBuffer::Samples::const_iterator i1) {
      std::cout << "writer_iq::process_iq " << service->approx_ptime() << " "
                << pos_ << " " 
                << std::distance(i0, i1) << std::endl;
      const boost::filesystem::path filepath(filename_);
      if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
        std::cerr << "file '" << filepath << "' exists and will be overwritten" << std::endl;
        boost::filesystem::remove(filepath);
      }
      if (not boost::filesystem::exists(filepath)) {
        // boost::filesystem::ofstream ofs(filepath, std::ios::binary);
        std::cerr << "new file " << filepath << " samplerate= " 
                  << service->sample_rate_hz() << std::endl;
        std::ofstream ofs(filename_.c_str(), std::ios::binary);        
        writeT(ofs, header_.change_sample_rate(service->sample_rate_hz()));
        pos_ = ofs.tellp();
        std::cerr << "writing header to file '" << filepath << "' exists " << std::endl;
      }
      // write data
      // TBD
      std::ofstream ofs(filename_.c_str(), std::ios::in | std::ios::out | std::ios::binary);
      ofs.seekp(pos_);
      for (IQBuffer::Samples::const_iterator i=i0; i!=i1; ++i) {
        write_real_sample(ofs, header_.format().bitsPerSample(), i->real()); // i
        write_real_sample(ofs, header_.format().bitsPerSample(), i->imag()); // q
      }
      pos_ = ofs.tellp();      
      writeT(ofs.seekp(0), header_.add_samples(std::distance(i0, i1)));
    }

  protected:
  private:
    std::string    filename_;
    std::streampos pos_;    
    wave_header    header_;
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
#if 0
  test_proc p;
  wave::reader_iq<test_proc> wr(p, 0.0, boost::posix_time::ptime(), 1.0, 0.0);
#else
  wave::writer_iq p("ttest.wav", 0, 16, 2);
  wave::reader_iq<wave::writer_iq> wr(p, 0.0, boost::posix_time::ptime(), 1.0, 0.0);
#endif
  wr.process_file("wav/iq_300kHz.wav");
  wr.finish();
  std::cout << "samples  " << wr.samples().size() << std::endl;

  // wave::reader_perseus<test_proc> wrp(p, 1.0, 0.0);
  return 0;
}
