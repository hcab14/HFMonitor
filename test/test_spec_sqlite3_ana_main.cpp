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
#include <boost/program_options.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <bzlib.h>

#include "FFT.hpp"
#include "Spectrum.hpp"
#include "logging.hpp"
// #include "carrier_monitoring/background_estimator.hpp"

#include "db/sqlite3.hpp"

std::ostream& operator<<(std::ostream& os, const std::vector<float>& v) {
  os << "[";
  std::copy(v.begin(), v.end(), std::ostream_iterator<float>(os, " "));
  return os << "]";
}
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

      // std::string st;
      // st =
      //   "PRAGMA page_size = 65536; ";
      // db::sqlite3::statement(connection_, st).step_all();
      // st =
      //   "PRAGMA foreign_keys = ON; ";
      // db::sqlite3::statement(connection_, st).step_all();
      // st =
      //   "CREATE TABLE IF NOT EXISTS SpecInfo (specId INTEGER PRIMARY KEY, tMin, tMax, nSpec INTEGER DEFAULT 0, fMin, fMax, df);";
      // db::sqlite3::statement(connection_, st).step_all();
      // st=
      //   "CREATE TABLE IF NOT EXISTS SpecData (t, pMin, pMax, s BLOB, specId INTEGER, FOREIGN KEY(specId) REFERENCES SpecInfo(specId));";
      // db::sqlite3::statement(connection_, st).step_all();
      // st =
      //   "CREATE TRIGGER IF NOT EXISTS trig AFTER INSERT ON SpecData \n"
      //   "\tBEGIN \n"
      //   "\t\tUPDATE SpecInfo SET tMax  = NEW.t   WHERE specId==NEW.specId; \n"
      //   "\t\tUPDATE SpecInfo SET nSpec = nSpec+1 WHERE specId==NEW.specId; \n"
      //   "\t\tUPDATE SpecInfo SET tMin  = NEW.t   WHERE specId==NEW.specId AND tMin IS NULL; \n"
      //   "\tEND;\n";
      // db::sqlite3::statement(connection_, st).step_all();

      // std::cout << "XXXX " << ps.deltaf() << " " << ps.size() << std::endl;

      // st = str(boost::format("INSERT INTO SpecInfo (fMin,fMax,df) VALUES (%.6f, %.6f, %.9f);") % ps.fmin() % ps.fmax() % ps.deltaf());
      // db::sqlite3::statement(connection_, st).step_all();

      // specId_ = connection_->last_insert_rowid();

      // stmt_.init(connection_,  "INSERT INTO SpecData (t,pMin,pMax,s,specId) VALUES ($time, $pMin, $pMax, $s, $specId);");
    // const int result = BZ2_bzBuffToBuffCompress(reinterpret_cast<char*>(&vc_[0]), &dest_len,
    //                                             reinterpret_cast<char*>(&v_[0]),  v_.size(),
    //                                             9, 0, 0);

#if 0
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
#endif

class spec_buf {
public:
  typedef std::vector<float> vec_type;
  typedef vec_type::iterator iterator;
  typedef vec_type::const_iterator const_iterator;

  spec_buf(double f_min,
           double f_max,
           double df,
           int    nt,
           double dt)
    : f_min_(f_min)
    , f_max_(f_max)
    , df_(df)
    , nf_(int((f_max_-f_min_)/df+0.5)+1)
    , nt_(nt)
    , dt_(dt)
    , spec_(nf_*nt_, 0.0f)
    , spec_norm_(nf_*nt_, 0.0f) {
    std::cout << "f_min,max= " << f_min_ << " " << f_max_
              << " df,nf= " << df_ << " " << nf_
              << " dt,nt= " << dt_ << " " << nt_
              << " size: " << spec_.size() << " " << spec_norm_.size()
              << std::endl;
  }

  double f_min() const { return f_min_; }
  double f_max() const { return f_max_; }
  double df()    const { return df_;    }
  int    nf()    const { return nf_;    }
  double dt()    const { return dt_;    }
  int    nt()    const { return nt_;    }

  void insert(int it, std::vector<double>::const_iterator si) {
    std::cout << "insert it= " << it << " " << nf() << std::endl;
    for (int i=0; i<nf(); ++i, ++si)
      spec_[it + i*nt_] = *si;
  }
  iterator get(int jf)      { return spec_.begin()      + jf*nt_; }
  iterator get_norm(int jf) { return spec_norm_.begin() + jf*nt_; }

  void clear() { spec_.clear(); }

  void proc() {
    normalize();
    for (int jf=0; jf<nf(); ++jf) {
      iterator snp_begin = get_norm(jf);
      vec_type xc = corr(snp_begin, snp_begin+nt());
      //      std::cout << "corr= " << xc << std::endl;
      iterator imc = std::max_element(xc.begin()+4*16, xc.end());
      const float max_corr = *imc;
      std::cout << str(boost::format("proc: f=%f max_corr=%4.2f p=%f") % (f_min()+jf*df()) % max_corr % (std::distance(xc.begin(), imc)/16.0f)) << std::endl;
      if (max_corr < 0.4)
        continue;

      float period         = compute_period(snp_begin, snp_begin+nt(), xc, imc);
      float period_precise = compute_period_precise(xc, period);
      bool  is_ok          = check_signal(xc, period_precise);
      std::cout << str(boost::format("proc: f=%f max_corr=%4.2f period=%f,%f is_ok=%d")
                       % (f_min()+jf*df())
                       % max_corr
                       % (period/16)
                       % (period_precise/16)
                       % is_ok) << std::endl;
    }
  }

  float compute_duty_cycle(const_iterator i0, const_iterator i1, int p) const {
    float sum     = 0.0f;
    int   counter = 0;
    for (const_iterator i=i0; i!=i1 && i!=i0+p; ++i) {
      sum     += ((*i) < 0.0);
      counter += 1;
    }
    return (counter ? sum / counter : 0.5);
  }

  float compute_period(const_iterator i0, const_iterator i1, vec_type& xc, iterator imc) {
    const float dc  = compute_duty_cycle(i0, i1, std::distance(xc.begin(), imc));
    const float dcy = 1-4*(dc < 0.5 ? dc : 1-dc);
    vec_type xxc(xc.size(), 0);
    for (int i=0; i<xc.size(); ++i)
      xxc[i] = (2*xc[i]-dcy-1) / (1-dcy);

    int ip = std::distance(xc.begin(), imc);
    float_t min_period = 4*16;
    int m = std::floor(ip/min_period);
    vec_type y(m,0);
    for (int i=0; i<m; ++i) {
      for (int j=0; j<i+1; ++j) {
        y[i] += xxc[int(0.5 + ip*float(j+1)/float(i+1))];
      }
    }
    iterator im = std::max_element(y.begin(), y.end());
    const float period = float(ip)/float(1+std::distance(y.begin(), im));
    xc = xxc;
    return period;
  }

  bool check_signal(const vec_type& xc, float period) {
    // check
    const int m = std::floor(xc.size()/period);
    float sum[2] = { 0, 0 };
    for (int i=0; i<m; ++i) {
      const int idx_max = int((i+1.0)*period + 0.5);
      const int idx_min = int((i+0.0)*period + 0.5);
      const_iterator i_min = std::min_element(xc.begin()+idx_min,
                                              xc.begin()+idx_max);
      sum[0] += *i_min;
      sum[1] += xc[idx_max];
      std::cout << "sum: " << period << " " << i << " "
                << *i_min << " " << xc[idx_max]
                << std::endl;
    }
    sum[0] /= m;
    sum[1] /= m;
    std::cout << "sum: " << sum[0] << " " << sum[1] << std::endl;
    return (sum[0] < 0 &&
            sum[1] > 0 &&
            sum[1]-sum[0] > 0.2);
  }

  float compute_period_precise(const vec_type& xc, float period) const {
    const int m = std::floor(xc.size()/period - 0.5);
    vec_type ps(4*m+1, 0);
    for (int i=-2*m; i<=2*m; ++i)
      ps[2*m+i] = int(period+0.5) + float(i)/float(m);

    vec_type y(ps.size(), 0);
    for (int i=0; i<4*m+1; ++i) {
      y[i] = 0.0f;
      for (int j=1; j<=m; ++j) {
        y[i] += xc[int(j*ps[i] + 0.5)];
      }
      y[i] /= m;
      std::cout << "compute_period_precise: " << i << " " << ps[i] << " " << y[i] << std::endl;
    }
    iterator im = std::max_element(y.begin(), y.end());
    return ps[std::distance(y.begin(), im)];
  }
  vec_type corr(const_iterator i0,
                const_iterator i1) const {
    int m = std::distance(i0, i1)/2;
    vec_type xc(m, 0);
    vec_type yc(m, 0);
    for (int i=0; i<m; ++i) {
      xc[i] = 0.0;
      for (int j=0; j<m; ++j) {
        xc[i] += (*(i0+j)) * (*(i0+i+j));
        yc[i] += (*(i0+i+j)) * (*(i0+i+j));
      }
    }
    const float norm = std::sqrt(xc[0]);
    for (int i=0; i<m; ++i) {
      xc[i] /= (norm*std::sqrt(yc[i]));
      if (std::abs(xc[i]) > 1)
        std::cout << "XXX " << xc[i] << std::endl;
    }
    return xc;
  }

  float quantile(const_iterator i0,
                 const_iterator i1,
                 float q) const {
    vec_type v(i0, i1);
    const int m = int(q*std::distance(i0, i1) + 0.5);
    std::nth_element(v.begin(), v.begin()+m, v.end());
    return v[m];
  }
  void normalize() {
    for (int jf=0; jf<nf(); ++jf) {
      iterator sp_begin  = get(jf);
      iterator snp_begin = get_norm(jf);

      std::cout << "normalize: f=" << f_min() + df()*jf << std::endl;

      float s[2] = {
        quantile(sp_begin, sp_begin+nt(), 0.1),
        quantile(sp_begin, sp_begin+nt(), 0.9)
        // *std::min_element(sp_begin, sp_begin+nt()),
        // *std::max_element(sp_begin, sp_begin+nt())
      };
      for (int k=0; k<20; ++k) {
        float sp[2] = { 0, 0 };
        int   np[2] = { 0, 0 };
        float thr = 0.5*(s[0]+s[1]);
        iterator isp = sp_begin;
        for (int l=0; l<nt(); ++l, ++isp) {
          const bool b(*isp > thr);
          sp[b] += *isp;
          np[b] += 1;
        }
        sp[0] /= np[0];
        sp[1] /= np[1];
        std::cout << "normalize k=" << k << " s0,s1=" << sp[0] << " " << sp[1] << " thr=" << thr << std::endl;
        if (std::abs(sp[0]-s[0]) < 0.1 &&
            std::abs(sp[1]-s[1]) < 0.1) {
          break;
        }
        s[0] = sp[0];
        s[1] = sp[1];
      }
      iterator isp  = sp_begin;
      iterator isnp = snp_begin;
      std::cout << "N ";
      for (int l=0; l<nt(); ++l, ++isp, ++isnp) {
        *isnp = 2*(*isp - s[0])/(s[1] - s[0]) - 1.0f;
        *isnp = (*isnp < -2.0f ? -2.0f : *isnp);
        *isnp = (*isnp > +2.0f ? +2.0f : *isnp);
        std::cout << " " << *isnp;
      }
      std::cout << std::endl;
    }
  }
protected:

private:
  double f_min_;
  double f_max_;
  double df_;
  int    nf_;
  int    nt_;
  double dt_;
  vec_type spec_;
  vec_type spec_norm_;
};

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_spec_sqlite3_ana");

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",    "produce help message")
    ("version,v", "display version")
    ("fMin_kHz",   po::value<double>()->default_value(   0),   "fMin (kHz)")
    ("fMax_kHz",   po::value<double>()->default_value(  40e3), "fMax (kHz)");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }
    if (vm.count("version")) {
      std::cout << SVN_VERSION_STRING << std::endl;
      return EXIT_SUCCESS;
    }

    const double fMin_kHz = vm["fMin_kHz"].as<double>();
    const double fMax_kHz = vm["fMax_kHz"].as<double>();

    db::sqlite3::connection::sptr connection = db::sqlite3::connection::make("file:test.db");
    db::sqlite3::statement        stmt;
    stmt.init(connection,
              "SELECT t,pMin,pMax,s,fMin,fMax,df "
              "FROM SpecData JOIN SpecInfo USING (specId)"
              "WHERE t>\"2016-10-22 14:35\" and t<=\"2016-10-22 14:37\";");

    std::vector<unsigned char> v(125*1000, 0);
    std::vector<double>        s(125*1000, 0);

    typedef db::sqlite3::statement::ptime ptime;

    spec_buf sb(1e3*fMin_kHz, 1e3*fMax_kHz, 4.0, 16*120, 0.25);

    int counter=0;
    while (stmt.step()) {
      const ptime          time   = stmt.get_column<ptime>(0);
      const double         pMin   = stmt.get_column<double>(1);
      const double         pMax   = stmt.get_column<double>(2);
      const unsigned char* bytes  = reinterpret_cast<const unsigned char*>(stmt.column_blob(3));
      const unsigned int   nBytes = stmt.column_bytes(3);
      const double         fMin   = stmt.get_column<double>(4);
      const double         fMax   = stmt.get_column<double>(5);
      const double         df     = stmt.get_column<double>(6);

      unsigned int dest_len = v.size();
      // BZ2_bzBuffToBuffDecompress(reinterpret_cast<char*>(&v[0]), &dest_len,
      //                            const_cast      <char*>(bytes),  nBytes,
      //                            0, 0);
      std::cout << "time= " << time
                << " pMin,pMax= "  << pMin << " " << pMax
                << " spec bytes= " << nBytes << "," << dest_len
                << " fMin,Max,df= " << fMin << " " << fMax << " " << df
                << std::endl;

      for (int i=0; i<nBytes; ++i)
        s[i] = pMin + (pMax-pMin)*bytes[i]/255.0;

      double f0 = 1e3*fMin_kHz;
      double f1 = 1e3*fMax_kHz;
      int    i0 = int((f0-fMin)/df+0.5);
      int    i1 = int((f1-fMin)/df+1.5);
      std::cout << "S ";
      for (int i=i0; i<i1; ++i)
        std::cout << s[i] << " ";
      std::cout << std::endl;;
      sb.insert(counter++, s.begin()+i0);
    }
    sb.proc();

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
