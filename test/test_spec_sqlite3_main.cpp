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
#include <cmath>
#include <deque>
#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "FFT.hpp"
#include "Spectrum.hpp"
#include "logging.hpp"
#include "wave/reader.hpp"
#include "repack_processor.hpp"
#include "run.hpp"
#include "carrier_monitoring/background_estimator.hpp"

#include "db/sqlite3.hpp"

std::ostream& operator<<(std::ostream& os, const std::vector<double>& v) {
  os << "[";
  std::copy(v.begin(), v.end(), std::ostream_iterator<double>(os, " "));
  return os << "]";
}
std::ostream& operator<<(std::ostream& os, const std::map<double, double>& v) {
  os << "[ ";
  for (std::map<double,double>::const_iterator i(v.begin()), iend(v.end()); i!=iend; ++i)
    os << "(" << i->first << "," << i->second << ") ";
  return os << "]";
}


class test_proc :  processor::base_iq  {
public:
  typedef boost::shared_ptr<test_proc> sptr;
  typedef FFT::FFTWTransform<double> fft_type;
  typedef boost::shared_ptr<fft_type> fft_sptr;
  typedef frequency_vector<double> fv_type;
  typedef boost::posix_time::time_duration time_duration;
  

  test_proc(const boost::property_tree::ptree& config)
    : base_iq(config)
    , f_min_Hz_(config.get<double>("<xmlattr>.fMin_Hz"))
    , f_max_Hz_(config.get<double>("<xmlattr>.fMax_Hz"))
    , specId_(0)
  {}

  void process_iq(processor::service_iq::sptr sp, const_iterator i0, const_iterator i1) {

    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;


    const ssize_t length(std::distance(i0, i1));
    if (!fftw_) {
      std::cout << "FFTW create" << std::endl;
      fftw_ = fft_sptr(new fft_type(length, FFTW_BACKWARD, FFTW_ESTIMATE));
      std::cout << "FFTW created" << std::endl;
    }

    if (length != ssize_t(fftw_->size())) {
      std::cout << "FFTW resize " << length << std::endl;
      fftw_->resize(length);
      std::cout << "FFTW resized" << std::endl;
    }
    
    fftw_->transformRange(i0, i1, FFT::WindowFunction::Blackman<float>(length));

    const FFTSpectrum<fft_type> s(*fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double offset_ppb(sp->offset_ppb());
    fv_type ps(f_min_Hz_, f_max_Hz_, s, std::abs<double>, offset_ppb);
    ps.apply(s2db());

    if (!connection_) {
      connection_ = db::sqlite3::connection::make("file:test.db?mode=rwc");
      std::string st;
      st =
        "PRAGMA page_size = 65536; ";
      db::sqlite3::statement(connection_, st).step_all();
      st = 
        "PRAGMA foreign_keys = ON; ";
      db::sqlite3::statement(connection_, st).step_all();
      st = 
        "CREATE TABLE IF NOT EXISTS SpecInfo (specId INTEGER PRIMARY KEY, tMin, tMax, nSpec INTEGER DEFAULT 0, fMin, fMax, df);";
      db::sqlite3::statement(connection_, st).step_all();
      st= 
        "CREATE TABLE IF NOT EXISTS SpecData (t, pMin, pMax, s BLOB, specId INTEGER, FOREIGN KEY(specId) REFERENCES SpecInfo(specId));";
      db::sqlite3::statement(connection_, st).step_all();
      st = 
        "CREATE TRIGGER IF NOT EXISTS trig AFTER INSERT ON SpecData \n"
        "\tBEGIN \n"
        "\t\tUPDATE SpecInfo SET tMax  = NEW.t   WHERE specId==NEW.specId; \n"
        "\t\tUPDATE SpecInfo SET nSpec = nSpec+1 WHERE specId==NEW.specId; \n"
        "\t\tUPDATE SpecInfo SET tMin  = NEW.t   WHERE specId==NEW.specId AND tMin IS NULL; \n"
        "\tEND;\n";
      db::sqlite3::statement(connection_, st).step_all();

      std::cout << "XXXX " << ps.deltaf() << " " << ps.size() << std::endl;

      st = str(boost::format("INSERT INTO SpecInfo (fMin,fMax,df) VALUES (%.6f, %.6f, %.9f);") % ps.fmin() % ps.fmax() % ps.deltaf());
      db::sqlite3::statement(connection_, st).step_all();

      specId_ = connection_->last_insert_rowid();

      stmt_.init(connection_,  "INSERT INTO SpecData (t,pMin,pMax,s,specId) VALUES ($time, $pMin, $pMax, $s, $specId);");
    }

    const double pMin(std::min_element(ps.begin(), ps.end(), fv_type::cmpSecond)->second);
    const double pMax(std::max_element(ps.begin(), ps.end(), fv_type::cmpSecond)->second);
    std::vector<unsigned char> v(ps.size());
    for (size_t i(0), n(ps.size()); i<n; ++i)
      v[i] = static_cast<unsigned char>(255.0*(ps[i].second-pMin)/(pMax-pMin));
    
    try {
      stmt_
        .bind("$time",   sp->approx_ptime())
        .bind("$pMin",   pMin)
        .bind("$pMax",   pMax)
        .bind_blob("$s", &v[0], v.size())
        .bind("$specId", specId_);
      stmt_.step_all();
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      LOG_ERROR(e.what()); 
    }
  }

  struct s2db {
    double operator()(double c) const {
      return 10*std::log10(c);
    }
  } ;
  template<typename T>
  double mean(const T& v) const {
    return mean(v.begin(), v.end());
  }
  template<class Iterator>
  double mean(Iterator beg, Iterator end) const {
    return std::accumulate(beg, end, typename Iterator::value_type(0))/(beg==end ? 1. : double(std::distance(beg, end)));
  }

  template<class Iterator>
  double mean_notzero(Iterator beg, Iterator end) const {
    double sum_1(0), sum_x(0);
    for (Iterator i(beg); i!=end; ++i) {
      if (*i) {
        sum_1 += 1;
        sum_x += *i;
      }
    }
    return sum_1 != 0 ? sum_x/sum_1 : 0.;
  }

  template<typename T>
  double median(const T& v) const {
    T vc(v);
    return median(vc.begin(), vc.end());
  }
  template<class Iterator>
  double median(Iterator beg, Iterator end) const {
    if (beg == end) return 0.;
    typedef typename Iterator::value_type value_type;
    const ssize_t mid(std::distance(beg, end)/2);

    std::nth_element(beg, beg+mid, end);
    const value_type v(*(beg+mid));

    if ((std::distance(beg,end)%2) == 1)
      return v;

    std::nth_element(beg, beg+mid-1, end);
    const value_type v2(*(beg+mid-1));

    return 0.5*double(v+v2);
  }


private:
  fft_sptr        fftw_;
  double          f_min_Hz_;
  double          f_max_Hz_;
  db::sqlite3::connection::sptr connection_;
  db::sqlite3::statement        stmt_;
  boost::int64_t  specId_;
} ;

int main(int argc, char* argv[])
{

  LOGGER_INIT("./Log", "test_spec_sqlite3");
  try {
    const boost::program_options::variables_map
      vm(process_options("config/multi_downconvert.xml", argc, argv));

    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    wave::reader_iq<test_proc> r(config.get_child("Test"));
    for (int i((argc == 1) ? 1 : 3); i<argc; ++i) {
      std::cout << "processing " << argv[i] << std::endl;
      r.process_file(argv[i]);
    }
    r.finish();

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what()); 
    return 1;
  }
  return 0;
}

