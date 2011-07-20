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
#include <boost/shared_ptr.hpp>
#include <time.h>

namespace wave {
  namespace chunk {
    // chunks are read in by using
    template<typename T>
    T readT(std::istream& is) {
      T data;
      is.read((char *)(&data), sizeof(T));
      if (!is) throw std::runtime_error("read failed");
      return data;
    }

    class header {
    public:
      typedef boost::shared_ptr<header> sptr;
      header(std::string id="XXXX",
             size_t size=0)
        : size_(size) {
        std::copy(id.begin(), id.begin()+std::min(size_t(4), id.size()), &id_[0]);
      }      
      std::string     id() const   { return std::string(id_, id_+4); }
      boost::uint32_t size() const { return size_; }
      friend std::ostream& operator<<(std::ostream& os, const header& h) {
        return os << "id=" << h.id() << " size=" << h.size();
      }
    private:
      boost::uint8_t  id_[4];
      boost::uint32_t size_;
    } ;
    
    class riff : public header {
    public:
      typedef boost::shared_ptr<riff> sptr;
      riff(size_t size=0)
        : header("RIFF", size) {
        std::string id2("WAVE");
        std::copy(id2.begin(), id2.begin()+std::min(size_t(4), id2.size()), &id2_[0]);
      }

      std::string id2() const { return std::string(id2_, id2_+4); }

      friend std::ostream& operator<<(std::ostream& os, const riff& r) {
        return os << header(r) << " id2=" << r.id2();
      }
      bool ok() const { return id() == "RIFF" && id2() == "WAVE"; }
    private:
      boost::uint8_t id2_[4]; 
    } ;
    
    class format : public header {
    public:
      typedef boost::shared_ptr<format> sptr;
      format(boost::uint32_t sampleRate=1,
             boost::uint16_t bitsPerSample=16,
             boost::uint16_t numChannels=2)
        : header("fmt ", sizeof(format))
        , audioFormat_(1)      // PCM format
        , numChannels_(numChannels)
        , sampleRate_(sampleRate)
        , bytesPerSecond_(numChannels*sampleRate*bitsPerSample/8)
        , bytesPerSample_(numChannels*bitsPerSample/8)
        , bitsPerSample_(bitsPerSample) {}

      boost::uint16_t audioFormat() const { return audioFormat_; }
      boost::uint16_t numChannels() const { return numChannels_; }
      boost::uint32_t sampleRate() const { return sampleRate_; }
      boost::uint32_t bytesPerSecond() const { return bytesPerSecond_; }
      boost::uint16_t bytesPerSample() const { return bytesPerSample_; }
      boost::uint16_t bitsPerSample() const { return bitsPerSample_; }
      bool ok() const { return id() == "fmt "; }
      
      friend std::ostream& operator<<(std::ostream& os, const format& f) {
        return os << header(f)
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
    
    class rcvr : public header {
    public:
      typedef boost::shared_ptr<rcvr> sptr;
      
      rcvr(boost::uint32_t nCenterFrequencyHz=0,
           boost::uint32_t nSamplingRateIdx=0,
           time_t          timeStart=0,
           boost::uint8_t  wAttenId=0,
           boost::uint8_t  bAdcPresel=0,
           boost::uint8_t  bAdcPreamp=0,
           boost::uint8_t  bAdcDither=0) 
        : header("rcvr", sizeof(rcvr))
        , nCenterFrequencyHz_(nCenterFrequencyHz)
        , nSamplingRateIdx_(nSamplingRateIdx)
        , timeStart_(timeStart)
        , wAttenId_(wAttenId)
        , bAdcPresel_(bAdcPresel)
        , bAdcPreamp_(bAdcPreamp)
        , bAdcDither_(bAdcDither)
        , bSpare_(0) {
        for (size_t i=0; i<16; ++i) rsrvd_[i]=0;
      }
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
      
      friend std::ostream& operator<<(std::ostream& os, const rcvr& r) {
        os << header(r) 
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
    
    class data: public header {
    public:
      typedef boost::shared_ptr<data> sptr;
      data()
        : header("data") {}

      friend std::ostream& operator<<(std::ostream& os, const data& d) {
        return os << header(d);
      }
      bool ok() const { return id() == "data"; }
    } ;    
  } // namespace chunk
} // namespace wave
#endif // _WAVE_HPP_cm100929_
