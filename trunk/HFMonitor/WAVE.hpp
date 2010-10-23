// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _WAVE_HPP_cm100929_
#define _WAVE_HPP_cm100929_
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/integer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <time.h>

#include "protocol.hpp"
#include "IQBuffer.hpp"

namespace WAVE {
  namespace Chunk {
    class Header {
    public:
      std::string     id() const   { return std::string(id_, id_+4); }
      boost::uint32_t size() const { return size_; }
      friend std::ostream& operator<<(std::ostream& os, const Header& h) {
        return os << "id=" << h.id() << " size=" << h.size();
      }
    private:
      boost::uint8_t  id_[4];
      boost::uint32_t size_;
    } ;
    
    class RIFF : public Header {
    public:
      std::string id2() const { return std::string(id2_, id2_+4); }

      friend std::ostream& operator<<(std::ostream& os, const RIFF& r) {
        return os << Header(r) << " id2=" << r.id2();
      }
      bool ok() const { return id() == "RIFF" && id2() == "WAVE"; }
    private:
      boost::uint8_t id2_[4]; 
    } ;
    
    class Format : public Header {
    public:
      boost::uint16_t audioFormat() const { return audioFormat_; }
      boost::uint16_t numChannels() const { return numChannels_; }
      boost::uint32_t sampleRate() const { return sampleRate_; }
      boost::uint32_t bytesPerSecond() const { return bytesPerSecond_; }
      boost::uint16_t bytesPerSample() const { return bytesPerSample_; }
      boost::uint16_t bitsPerSample() const { return bitsPerSample_; }
      bool ok() const { return id() == "fmt "; }
      
      friend std::ostream& operator<<(std::ostream& os, const Format& f) {
        return os << Header(f)
                  << " audioFormat=" << f.audioFormat()
                  << " numChannels=" << f.numChannels()
                  << " sampleRate=" << f.sampleRate()
                  << " bytesPerSecond=" << f.bytesPerSecond()
                  << " bytesPerSample=" << f.bytesPerSample()
                  << " bitsPerSample=" << f.bitsPerSample();
      }      
    private:
      boost::uint16_t audioFormat_;
      boost::uint16_t numChannels_;
      boost::uint32_t sampleRate_;
      boost::uint32_t bytesPerSecond_;
      boost::uint16_t bytesPerSample_;
      boost::uint16_t bitsPerSample_;
    } ;
    
    class Rcvr : public Header {
    public:
      boost::uint32_t nCenterFrequencyHz() const { return nCenterFrequencyHz_; }
      boost::uint32_t nSamplingRateIdx() const { return nSamplingRateIdx_; }
      boost::posix_time::ptime ptimeStart() const { return boost::posix_time::from_time_t(timeStart_); }
      boost::uint16_t wAttenId() const { return wAttenId_; }
      boost::uint8_t  bAdcPresel() const { return bAdcPresel_; }
      boost::uint8_t  bAdcPreamp() const { return bAdcPreamp_; }
      boost::uint8_t  bAdcDither() const { return bAdcDither_; }
      boost::uint8_t  bSpare() const { return bSpare_; }
      std::string     rsrvd() const { return std::string(rsrvd_, rsrvd_+16); }

      bool ok() const { return id() == "rcvr"; }
      
      friend std::ostream& operator<<(std::ostream& os, const Rcvr& r) {
        os << Header(r) 
           << " nCenterFrequencyHz=" << r.nCenterFrequencyHz()
           << " nSamplingRateIdx=" << r.nSamplingRateIdx()
           << " ptimeStart=" << r.ptimeStart()
           << " wAttenId=" << r.wAttenId()
           << " bAdcPresel=" << int(r.bAdcPresel())
           << " bAdcPreamp=" << int(r.bAdcPreamp())
           << " bAdcDither=" << int(r.bAdcDither())
           << " bSpare=" << int(r.bSpare())
           << " rsvrd="; std::copy(r.rsrvd_, r.rsrvd_+16, std::ostream_iterator<int>(os, " "));
        return os;
      }
    private:
      boost::uint32_t    nCenterFrequencyHz_; // center freq in Hz
      boost::uint32_t    nSamplingRateIdx_;   // 0=125k, 1=250k, 2=500k, 1 M 
      time_t             timeStart_;          // UTC rec time init
      boost::uint16_t    wAttenId_;           // 0=0dB, 1=10dB, 2=20dB, 3=30dB
      boost::uint8_t     bAdcPresel_;         // 0=Presel Off, 1=Presel On
      boost::uint8_t     bAdcPreamp_;         // 0=ADC Preamp Off, 1=ADC Preamp ON
      boost::uint8_t     bAdcDither_;         // 0=ADC Dither Off, 1=ADC Dither ON
      boost::uint8_t     bSpare_;             // for future use (default = 0)
      boost::uint8_t     rsrvd_[16];          // for future use (default = 000..0)
    } ;
    
    class Data: public Header {
    public:
      friend std::ostream& operator<<(std::ostream& os, const Data& d) {
        return os << Header(d);
      }
      bool ok() const { return id() == "data"; }
    } ;    
  } // namespace Chunk  

  template<typename PROCESSOR>
  class ProcessFile : private boost::noncopyable {
  public:
    typedef std::vector<std::complex<double> > SampleVec;
    ProcessFile(PROCESSOR& p,
                std::string fileName,
                boost::uint64_t sampleNumber,
                const std::vector<std::complex<double> >& samples,
                double bufferLengthSec, double overlap)
      : p_(p)
      , is_(fileName.c_str(), std::ios::in | std::ios::binary)
      , sampleNumber_(sampleNumber)
      , sampleNumberInFile_(-samples.size())
      , chunkRiff_(readT<Chunk::RIFF>  (is_))
      , chunkFmt_ (readT<Chunk::Format>(is_))
      , chunkRcvr_(readT<Chunk::Rcvr>  (is_))
      , chunkData_(readT<Chunk::Data>  (is_))
      , iqBuffer_(bufferLengthSec*chunkFmt_.sampleRate(), overlap) {
      iqBuffer_.insert(this, samples.begin(), samples.end());
    }
    ~ProcessFile() {}

    Chunk::RIFF     chunkRiff() const { return chunkRiff_; }
    Chunk::Format   chunkFmt()  const { return chunkFmt_; }
    Chunk::Rcvr     chunkRcvr() const { return chunkRcvr_; }
    Chunk::Data     chunkData() const { return chunkData_; }
    boost::uint64_t sampleNumber() const { return sampleNumber_; }
    std::vector<std::complex<double> > samples() const { return iqBuffer_.samples(); }
    
    // called from IQBuffer::insert
    void procIQ(IQBuffer::Samples::const_iterator i0, 
                IQBuffer::Samples::const_iterator i1) {
      using namespace boost::posix_time;
      // TO be checked:
      const time_duration 
        timeOffset(0,0,0, sampleNumberInFile_*time_duration::ticks_per_second()/chunkFmt().sampleRate());
      const Header 
        header(sampleNumber_ - iqBuffer_.n(),
               chunkFmt().sampleRate(), 
               chunkRcvr().nCenterFrequencyHz(),
               iqBuffer_.n(),
               0, // TODO
               chunkRcvr().wAttenId(),
               chunkRcvr().bAdcPresel(),
               chunkRcvr().bAdcPreamp(),
               chunkRcvr().bAdcDither(),
               chunkRcvr().ptimeStart() + timeOffset);
      std::cout << "WAVE::ProcessFile::procIQ: " << header << " "<< std::distance(i0,i1) << std::endl;
      p_.procIQ(header, i0, i1);
    }
    
    size_t proc() {
      size_t nRead(0);
      try {
        while (1) {
          const double xi(readRealSample());
          const double xq(readRealSample());
          iqBuffer_.insert(this, std::complex<double>(xq,xi));
          ++sampleNumber_;
          ++sampleNumberInFile_;
          ++nRead;
        }
      } catch (...) {
        std::cout << "End Of File" << std::endl;
      }
      return nRead;
    }
  protected:
  private:
    double readRealSample() {
      boost::uint8_t a(0);
      boost::int32_t sum(0);
      for (size_t u(0); u<chunkFmt().bitsPerSample(); u+=8) 
        sum |= ((a=readT<boost::uint8_t>(is_)) << u);
      const boost::int32_t iMax(1 << chunkFmt().bitsPerSample());
      return ((a&0x80) ? sum-iMax : sum) / double(iMax);
    }    
    template<typename T>
    static
    T readT(std::istream& is) {
      T data;
      is.read((char *)(&data), sizeof(T));
      if (!is) throw std::runtime_error("read failed");
      return data;
    }

    PROCESSOR&          p_;
    std::ifstream       is_;
    boost::uint64_t     sampleNumber_;
    boost::int64_t      sampleNumberInFile_;
    const Chunk::RIFF   chunkRiff_;
    const Chunk::Format chunkFmt_;
    const Chunk::Rcvr   chunkRcvr_;
    const Chunk::Data   chunkData_;
    IQBuffer            iqBuffer_;
  } ;
} // namespace WAVE
#endif // _WAVE_HPP_cm100929_
