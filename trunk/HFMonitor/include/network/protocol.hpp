// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _PROTOCOL_HPP_cm100625_
#define _PROTOCOL_HPP_cm100625_

#include <boost/cstdint.hpp>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "logging.hpp"

// -----------------------------------------------------------------------------
// generic header for any data type
//  * type, e.g. of form "IQNx", with  N=number of bytes, x=f/i (float/signed integer) 
//  * approx_ptime: approximate time stamp
//  * length: size of the following data packet in bytes
//
class header : private boost::noncopyable {
public:
  typedef boost::posix_time::ptime ptime;
  typedef boost::array<char, 4> id_type;

  header(std::string id="XX0x",
         ptime       approx_ptime=boost::posix_time::not_a_date_time,
         size_t      length=0)
    : approx_ptime_(approx_ptime)
    , length_(length) {
    ASSERT_THROW(id.size() == 4);
    std::copy(id.begin(), id.end(), id_);
  }

  std::string id()           const { return std::string(id_, id_+4); }
  ptime       approx_ptime() const { return approx_ptime_; }
  size_t      length()       const { return length_; }

  friend std::ostream& operator<<(std::ostream& os, const header& h) {
    return os << "'" << h.id() << "' "<< h.approx_ptime() << " len=" << h.length();
  }
protected:
private:
  char   id_[4];        // 4-byte identifier
  ptime  approx_ptime_; // approximate time tag
  size_t length_;       // length of following data
} ;

// -----------------------------------------------------------------------------
// information for sampled data
//  * num_channel
//  * bytes_per_channnel
//  * data_type
//  * length
//  * sample_rate_Hz
//  * ddc_center_frequecy_Hz

class sample_info : private boost::noncopyable {
public:
  sample_info(boost::uint32_t sample_rate_Hz=1,
              boost::uint32_t ddc_center_frequecy_Hz=0)
    : sample_rate_Hz_(sample_rate_Hz)
    , ddc_center_frequecy_Hz_(ddc_center_frequecy_Hz) {
  }
  virtual ~sample_info() {}
  
  double sample_rate_Hz()         const { return sample_rate_Hz_; }
  double ddc_center_frequecy_Hz() const { return ddc_center_frequecy_Hz_; }
protected:
private:
  boost::uint32_t sample_rate_Hz_;         // sample rate [Hz]
  boost::uint32_t ddc_center_frequecy_Hz_; // ddc center frequency [Hz]
} ;

class iq_info : public sample_info {
public:
  iq_info(boost::uint32_t sample_rate_Hz=1,
          boost::uint32_t ddc_center_frequecy_Hz=0,
          char sample_type,
          boost::uint8_t bytes_per_sample)
    : sample_info(sample_rate_Hz, ddc_center_frequecy_Hz)
    , sample_type_(sample_type)
    , bytes_per_sample_(bytes_per_sample_)
    , dummy_(0) {}
  virtual ~iq_info() {}

  char           sample_type()      const { return sample_type_; }
  boost::uint8_t bytes_per_sample() const { return bytes_per_sample_; }
protected:
private:
  char            sample_type_;
  boost::uint8_t  bytes_per_sample_;
  boost::uint16_t dummy_;
} ;


class Header {
public:
  Header(boost::int64_t  sampleNumber=0,
         boost::uint32_t sampleRate=1,
         boost::uint32_t ddcCenterFrequency=0,
         boost::uint32_t numberOfSamples=0,
         boost::uint8_t  samplingRateIndex=0,
         boost::uint8_t  attenId=0,
         boost::uint8_t  adcPresel=1,
         boost::uint8_t  adcPreamp=0,
         boost::uint8_t  adcDither=0,
         boost::posix_time::ptime approxPTime=boost::posix_time::ptime())
    : sampleNumber_      (sampleNumber)
    , sampleRate_        (sampleRate)
    , ddcCenterFrequency_(ddcCenterFrequency)
    , numberOfSamples_   (numberOfSamples)
    , samplingRateIndex_ (samplingRateIndex)
    , attenId_           (attenId)
    , adcPresel_         (adcPresel)
    , adcPreamp_         (adcPreamp)
    , adcDither_         (adcDither)
    , dummy_             (0)
    , approxPTime_       (approxPTime) {}

  boost::int64_t  sampleNumber()        const { return sampleNumber_; }
  boost::int64_t& sampleNumber()              { return sampleNumber_; }
  boost::uint32_t sampleRate()          const { return sampleRate_; }
  boost::uint32_t ddcCenterFrequency()  const { return ddcCenterFrequency_; }
  boost::uint32_t numberOfSamples()     const { return numberOfSamples_; }
  boost::uint8_t  samplingRateIndex()   const { return samplingRateIndex_; }
  boost::uint8_t  attenId()             const { return attenId_; }
  boost::uint8_t  adcPresel()           const { return adcPresel_; }
  boost::uint8_t  adcPreamp()           const { return adcPreamp_; }
  boost::uint8_t  adcDither()           const { return adcDither_; }
  boost::posix_time::ptime  approxPTime() const { return approxPTime_; }
  boost::posix_time::ptime& approxPTime() { return approxPTime_; }

  void setNumberOfSamples(boost::uint32_t n)  { numberOfSamples_ = n; }

  friend std::ostream& operator<<(std::ostream& os, const Header& h) {
    return os << "sampleNumber="        << h.sampleNumber() 
              << " approxPTime="        << h.approxPTime()
              << " sampleRate="         << h.sampleRate()
              << " ddcCenterFrequency=" << h.ddcCenterFrequency()
              << " numberOfSamples="    << h.numberOfSamples()
              << " samplingRateIndex="  << int(h.samplingRateIndex())
              << " attenId="            << int(h.attenId())
              << " adcPreamp="          << int(h.adcPreamp())
              << " adcDither="          << int(h.adcDither());
  }
  
  bool hasEqualParameters(const Header& h) const {
    return ( sampleRate_         == h.sampleRate_ &&
             ddcCenterFrequency_ == h.ddcCenterFrequency_ &&
             samplingRateIndex_  == h.samplingRateIndex_ &&
             attenId_            == h.attenId_ &&
             adcPresel_          == h.adcPresel_ &&
             adcPreamp_          == h.adcPreamp_ &&
             adcDither_          == h.adcDither_ );
  }

protected:
private:
  boost::int64_t           sampleNumber_;       // 8
  boost::uint32_t          sampleRate_;         // 4
  boost::uint32_t          ddcCenterFrequency_; // 4
  boost::uint32_t          numberOfSamples_;    // 4
  boost::uint8_t           samplingRateIndex_;  // 1
  boost::uint8_t           attenId_   :  5;     // 1
  boost::uint8_t           adcPresel_ :  1;
  boost::uint8_t           adcPreamp_ :  1;
  boost::uint8_t           adcDither_ :  1;
  boost::uint16_t          dummy_;              // 2
  boost::posix_time::ptime approxPTime_;        // 8
} ;

#endif // _PROTOCOL_HPP_cm100625_
