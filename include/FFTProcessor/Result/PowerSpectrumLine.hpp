// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
#ifndef _FFT_RESULT_POWERSPECTRUM_LINE_HPP_cm101026_
#define _FFT_RESULT_POWERSPECTRUM_LINE_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "Spectrum.hpp"
#include "FFTProcessor/Result.hpp"
#include "logging.hpp"
#include "netpbm.hpp"

namespace Result {
  class PowerSpectrumLine : public Base {
  public:
    typedef boost::shared_ptr<PowerSpectrumLine> sptr;
    typedef boost::shared_ptr<PowerSpectrumLine> Handle;
    typedef frequency_vector<double> PowerSpectrum;

    PowerSpectrumLine(ptime time, const PowerSpectrum&ps)
      : Base("PowerSpectrumLine", time) {
      std::string line;
      std::copy((char*)&time, (char*)&time+sizeof(time), std::back_inserter(line_));
      if (ps.empty())
        line_.append(2*sizeof(float), char(0));
      else {
        PowerSpectrum::const_iterator iMin(std::min_element(ps.begin(), ps.end(), PowerSpectrum::cmpSecond));
        const float slMin(std::log10(iMin->second));
        std::copy((char*)&slMin, (char*)&slMin+sizeof(slMin), std::back_inserter(line_));
        
        PowerSpectrum::const_iterator iMax(std::max_element(ps.begin(), ps.end(), PowerSpectrum::cmpSecond));
        const float slMax(std::log10(iMax->second));
        std::copy((char*)&slMax, (char*)&slMax+sizeof(slMax), std::back_inserter(line_));
        
        BOOST_FOREACH(const PowerSpectrum::value_type& fs, ps)
          line_.push_back((unsigned char)((std::log10(fs.second) - slMin)/(slMax-slMin)*255));
      }
    }

    ~PowerSpectrumLine() {}
    virtual std::string toString() const { 
      return Base::toString() + " size=" + boost::lexical_cast<std::string>(line_.size());
    }

    virtual std::string lineBreak() const { return ""; }
    virtual std::string fileExtension() const { return "pnm";  }
    virtual file_period filePeriod() const { return gen_filename::Period5Minutes; }
    virtual boost::filesystem::fstream& dumpHeaderToFile(boost::filesystem::fstream& os) const {
      netpbm::pgm_writer pw(line_.size(), os);
      pw.write_header();
      return os;
    }
    virtual boost::filesystem::fstream& dumpDataToFile(boost::filesystem::fstream& os) const {
      netpbm::pgm_writer pw(line_.size(), os);
      pw.read_header();
      ASSERT_THROW(pw.write_line(line_) == true);
      return os;
    }
    virtual void updateTimeTag(boost::filesystem::fstream& os,
                               std::ostream::streampos pos,
                               std::string timeTag) const {
      // NOP
    }
    virtual std::string format() const { return "PGM_0000"; }

  protected:
  private:
    std::string line_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_POWERSPECTRUM_LINE_HPP_cm101026_
