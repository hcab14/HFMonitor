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
#ifndef _PROTOCOL_HPP_cm100625_
#define _PROTOCOL_HPP_cm100625_

#include <boost/cstdint.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace protocol {
  namespace perseus {
    
    class header {
    public:
      header(boost::int64_t  sampleNumber=0,
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

      header resample(size_t num, size_t denum) const {
        header result(*this);
        result.sampleRate_ *= num;
        result.sampleRate_ /= denum;
        return result;
      }
      header freq_shift(double freq_shift_hz) const {
        header result(*this);
        result.ddcCenterFrequency_ += freq_shift_hz;
        return result;
      }

      friend std::ostream& operator<<(std::ostream& os, const header& h) {
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
      
      bool hasEqualParameters(const header& h) const {
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
  } // namespace perseus
} // namespace protocol
#endif // _PROTOCOL_HPP_cm100625_
