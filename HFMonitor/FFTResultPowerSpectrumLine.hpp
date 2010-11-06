// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_POWERSPECTRUM_LINE_HPP_cm101026_
#define _FFT_RESULT_POWERSPECTRUM_LINE_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "Spectrum.hpp"
#include "FFTResult.hpp"
#include "netpbm.hpp"

namespace Result {
  class PowerSpectrumLine : public Base {
  public:
    typedef boost::shared_ptr<PowerSpectrumLine> Handle;
    typedef frequency_vector<double> PowerSpectrum;
    typedef boost::posix_time::ptime ptime;

    PowerSpectrumLine(ptime t, const PowerSpectrum&ps)
      : Base("PowerSpectrumLine") {
      std::string line;
      std::copy((char*)&t, (char*)&t+sizeof(t), std::back_inserter(line_));
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
    virtual FilePeriod filePeriod() const { return Base::Period5Minutes; }
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {
      netpbm::pgm_writer pw(line_.size(), os);
      pw.write_header();
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      netpbm::pgm_writer pw(line_.size(), os);
      pw.read_header();
      pw.write_line(line_);
      return os;
    }
  protected:
  private:
    std::string line_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_POWERSPECTRUM_LINE_HPP_cm101026_
