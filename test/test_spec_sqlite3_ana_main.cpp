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
  typedef db::sqlite3::statement::ptime ptime;

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
    , spec_(nf_*nt_, 0.0f) {
    std::cout << "f_min,max= " << f_min_ << " " << f_max_
              << " df,nf= " << df_ << " " << nf_
              << " dt,nt= " << dt_ << " " << nt_
              << " size: " << spec_.size()
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

  void clear() { spec_.clear(); }

  struct signal {
    signal(int ip, int m)
      : _ip(ip)
      , _m(m)
      , _avg_sig(_ip, 0.0f)
      , _sig(_ip*_m, 0.0f) {
      std::cout << "signal: ip= " << ip << " m= " << m << std::endl;
    }

    int ip() const { return _ip; }
    int m()  const { return _m; }

    float& sig(int i, int j) { return _sig[i + _ip*j]; }
    float& avg_sig(int i)    { return _avg_sig[i]; }

    const vec_type& avg_sig() const { return _avg_sig; }
    const vec_type& sig()     const { return _sig; }

    int _ip;
    int _m;
    vec_type _avg_sig;
    vec_type _sig;
  } ;

  std::vector<char> encode(const std::vector<float> &v, float v_min, float v_max) const {
    return encode(v.begin(), v.end(), v_min, v_max);
  }
  std::vector<char> encode(const_iterator i0, const_iterator i1, float v_min, float v_max) const {
    const int n = std::distance(i0, i1);
    std::vector<char> cv(n, 0);
    for (int i=0; i<n; ++i) {
      float x = (i0[i]-v_min)/(v_max-v_min);
      x = (x > 1.0f ? 1.0f : (x < 0.0f ? 0.0f : x));
      cv[i] = char(127*(2*x - 1));
    }
    return cv;
  }
  std::vector<float> decode(const char* p, int n, float v_min, float v_max) const {
    std::vector<float> x(n, 0);
    for (int i=0; i<n; ++i) {
      x[i] = v_min + (v_max-v_min)*0.5*(float(p[i])/127+1);
    }
    return x;
  }
  void proc(ptime t) {
    proc_L0(t);
    proc_L1();
  }

  void proc_L1() {
    if (!connection_)
      connection_ = db::sqlite3::connection::make("file:test__L0.db?mode=rwc");

    db::sqlite3::statement stmtFindMaxCorr;
    stmtFindMaxCorr.init(connection_,
                         "SELECT sigId,freq_Hz,max_corr,sig_norm "
                         "FROM Signal_L0 "
                         "WHERE sig_idx == -1 "
                         "ORDER BY max_corr DESC"
                         ";");

    db::sqlite3::statement stmtFindNearSignalsUp;
    stmtFindNearSignalsUp.init(connection_,
                               "SELECT sigId,freq_Hz,max_corr,sig_norm "
                               "FROM Signal_L0 "
                               "WHERE (sig_idx == -1 AND freq_Hz < $freq0+5e3 AND freq_Hz > $freq0) "
                               "ORDER BY freq_Hz ASC "
                               ";");
    db::sqlite3::statement stmtFindNearSignalsDown;
    stmtFindNearSignalsDown.init(connection_,
                                 "SELECT sigId,freq_Hz,max_corr,sig_norm "
                                 "FROM Signal_L0 "
                                 "WHERE (sig_idx == -1 AND freq_Hz > $freq0-5e3 AND freq_Hz < $freq0) "
                                 "ORDER BY freq_Hz DESC "
                                 ";");

    db::sqlite3::statement stmtUpdateIdx;
    stmtUpdateIdx.init(connection_,
                       "UPDATE Signal_L0 "
                       "SET sig_idx = $idx, sig_idx_center = $idx_center "
                       "WHERE sigId == $sigId "
                       ";");

    bool success = true;
    for (int idx_counter=0; success; ++idx_counter) {
      success = stmtFindMaxCorr.step();
      if (!success)
        continue;
      const int   idx0      = stmtFindMaxCorr.get_column<int>(0);
      const float freq0     = stmtFindMaxCorr.get_column<float>(1);
      const float max_corr0 = stmtFindMaxCorr.get_column<float>(2);
      const char* bytes     = reinterpret_cast<const char*>(stmtFindMaxCorr.column_blob(3));
      const int   nBytes    = stmtFindMaxCorr.column_bytes(3);

      const vec_type sig_norm0 = decode(bytes, nBytes, -2.0f, 2.0f);

      std::cout << "FOUND: " << freq0 << " " << max_corr0 << " " << success << std::endl;
      stmtFindMaxCorr.reset();

      stmtUpdateIdx
        .bind("$sigId",      idx0)
        .bind("$idx",        idx_counter)
        .bind("$idx_center", idx_counter)
        .step_all();

      mark_signals(idx_counter, freq0, sig_norm0, stmtFindNearSignalsUp,   stmtUpdateIdx);
      mark_signals(idx_counter, freq0, sig_norm0, stmtFindNearSignalsDown, stmtUpdateIdx);
    }
  }

  void mark_signals(int idx_counter, float freq0,
                    const vec_type& sig_norm0,
                    db::sqlite3::statement& stmtFindNearSignals,
                    db::sqlite3::statement& stmtUpdateIdx) {
    stmtFindNearSignals.bind("$freq0", freq0);

    std::vector<int> vec_idx;
    float last_freq = freq0;
    bool do_skip = false;
    while (stmtFindNearSignals.step()) {
      if (do_skip)
        continue;
      const int   idx      = stmtFindNearSignals.get_column<int>(0);
      const float freq     = stmtFindNearSignals.get_column<float>(1);
      const float max_corr = stmtFindNearSignals.get_column<float>(2);
      const char* bytes    = reinterpret_cast<const char*>(stmtFindNearSignals.column_blob(3));
      const int   nBytes   = stmtFindNearSignals.column_bytes(3);

      const vec_type sig_norm = decode(bytes, nBytes, -2.0f, 2.0f);

      const float cc = xcorr(sig_norm0, sig_norm);
      const bool  match = (std::abs(freq-last_freq) < 8 && cc > 0.1);
      std::cout << "Near signals: " << freq0
                << " | " << freq << " " << last_freq
                << " | " << max_corr
                << " | " << cc << " "  << match << std::endl;
      if (match)
        vec_idx.push_back(idx);

      last_freq = freq;
      do_skip = !match;
    }
    for (int i=0, n=vec_idx.size(); i<n; ++i) {
      stmtUpdateIdx
        .bind("$sigId",      vec_idx[i])
        .bind("$idx",        idx_counter)
        .bind("$idx_center", -1)
        .step_all();
    }
  }

  const_iterator find_max_corr(const_iterator beg, const_iterator end, int idt) const {
    // exclude the range of max [0, dt]
    const_iterator i=beg, j=i+1;
    for (const_iterator jend=beg+idt; j!=jend; ++i, ++j) {
      if (*j - *i > 0 || *j < 0) break;
    }
    std::cout << str(boost::format("find_max_corr: t0=%d") % std::distance(beg, j)) << std::endl;
    return std::max_element(j, end);
  }

  void proc_L0(ptime t) {
    // make up L0 table
    if (!connection_) {
      connection_ = db::sqlite3::connection::make("file:test__L0.db?mode=rwc");
      std::string st;

      st = "PRAGMA foreign_keys = ON; ";
      db::sqlite3::statement(connection_, st).step_all();

      st =
        "CREATE TABLE IF NOT EXISTS Signal_L0 (sigId INTEGER PRIMARY KEY"
        ", freq_Hz"          // frequency
        ", max_corr"         // max. auto-correlation
        ", period_sec"       // signal period (sec)
        ", bkgd_dB"          //
        ", sn_dB"            //
        ", pMin_dB, pMax_dB" //
        ", sig_idx DEFAULT -1"        //
        ", sig_idx_center DEFAULT -1" //
        ", sig_orig BLOB"   // original signal nomalized to [pMin_dB, pMax_dB]
        ", sig_norm BLOB"   // normalized signal [-2..+2]
        ", sig_corr BLOB"   // correlation [-1..+1]
        ");";
      db::sqlite3::statement(connection_, st).step_all();

      stmtSig_.init(connection_,
                    "INSERT INTO Signal_L0 ( freq_Hz"
                    ", max_corr, period_sec, bkgd_dB"
                    ", sn_dB, pMin_dB, pMax_dB"
                    ", sig_orig, sig_norm, sig_corr) "
                    "VALUES ($freq_Hz"
                    ",$max_corr,$period_sec,$bkgd_dB"
                    ",$sn_dB,$pMin_dB,$pMax_dB"
                    ",$sig_orig,$sig_norm,$sig_corr) "
                    ";");
    }

    vec_type sig_norm(nt(), 0.0f);
    float_t pMinMax[2] = { 0,0 };
    for (int jf=0; jf<nf(); ++jf) {
      // normalize
      normalize(get(jf), sig_norm.begin(), pMinMax);

      // compute correlation
      const vec_type xc_L0 = corr(sig_norm.begin(), sig_norm.end());

      // find max. correlation exluding delta t < 4 sec
      const_iterator imc          = find_max_corr(xc_L0.begin(), xc_L0.end(), 4*16);
      const float    max_corr_L0  = *imc;
      const int      period_L0    = std::distance(xc_L0.begin(), imc);
      std::cout << str(boost::format("proc(L0): f=%.0f max_corr=%4.2f p=%5.2f (%3d)")
                       % (f_min()+jf*df())
                       % max_corr_L0
                       % (period_L0/16.0f)
                       % period_L0) << std::endl;

      // thr
      if (max_corr_L0 < 0.3 || period_L0 < 16*3.0f)
        continue;

      // compute duty cycle
      const float dc = compute_duty_cycle(sig_norm.begin(), sig_norm.begin()+period_L0);

      // compute scaled autocorrelation
      const vec_type xc_L1       = scale_autocorrelation(xc_L0, dc);
      const float    max_corr_L1 = xc_L1[period_L0];
      const float    period_L1   = compute_period(xc_L1, period_L0);

      if (max_corr_L1 < 0.3 || period_L1 < 16*3.0f)
        continue;

      std::cout << str(boost::format("proc(L1): f=%.0f max_corr=%4.2f p=%5.2f (%3d) dc=%.3f max_corr=%4.2f p=%6.3f")
                       % (f_min()+jf*df())
                       % max_corr_L0
                       % (period_L0/16.0f)
                       % period_L0
                       % dc
                       % max_corr_L1
                       % (period_L1/16.0f))
                << std::endl;

      iterator s_beg = get(jf);
      const float sig_min = *std::min_element(s_beg, s_beg+nt());
      const float sig_max = *std::max_element(s_beg, s_beg+nt());

      const std::vector<char> cv_sig     (encode(s_beg, s_beg+nt(), sig_min, sig_max));
      const std::vector<char> cv_sig_norm(encode(sig_norm, -2, 2));
      const std::vector<char> cv_sig_corr(encode(xc_L1,    -2, 2));

      stmtSig_
        .bind("$freq_Hz",    f_min() + jf*df())
        .bind("$max_corr",   max_corr_L1)
        .bind("$period_sec", period_L1/16.0f)
        .bind("$bkgd_dB",    pMinMax[0])
        .bind("$sn_dB",      pMinMax[1]-pMinMax[0])
        .bind("$pMin_dB",    sig_min)
        .bind("$pMax_dB",    sig_max)
        .bind_blob("$sig_orig", &cv_sig[0],      cv_sig.size())
        .bind_blob("$sig_norm", &cv_sig_norm[0], cv_sig_norm.size())
        .bind_blob("$sig_corr", &cv_sig_corr[0], cv_sig_corr.size());
      stmtSig_.step_all();
    }
  }

  signal make_avg_signal(const_iterator i0, const_iterator i1, float p) const {
    signal sig(int(p+0.5),
               std::floor(std::distance(i0, i1)/p));

    for (int i=0; i<sig.ip(); ++i) {
      for (int j=0; j<sig.m(); ++j) {
        const int   k = std::floor(i+j*p);
        const float x = i+j*p-k;
        const float s  = i0[k] + x*(i0[k+1] - i0[k]);
        sig.sig(i,j)    = s;
        sig.avg_sig(i) += s;
      }
      sig.avg_sig(i) /= sig.m();
    }
    return sig;
  }

  float compute_duty_cycle(const_iterator i0, const_iterator i1) const {
    float sum     = 0.0f;
    int   counter = 0;
    for (const_iterator i=i0; i!=i1; ++i) {
      sum     += ((*i) < 0.0f);
      counter += 1;
    }
    const float dc = (counter ? sum / counter : 0.5);
    return std::max(0.1f, std::min(0.9f, dc));
  }

  vec_type scale_autocorrelation(const vec_type& xc, float dc) const {
    const float dcy = 1-4*(dc < 0.5 ? dc : 1-dc);
    vec_type xxc(xc.size(), 0);
    for (int i=0; i<xc.size(); ++i)
      xxc[i] = (2*xc[i]-dcy-1) / (1-dcy);

    return xxc;
  }
  float compute_period(const vec_type& xc, int ip) const {
    const float_t min_period = 2; // 2/16 sec
    const int m = std::floor(ip/min_period);
    vec_type y(m,0);
    for (int i=0; i<m; ++i) {
      y[i] = 0.0f;
      for (int j=0; j<i+1; ++j) {
        const int k = int(0.5 + ip*float(j+1)/float(i+1));
        y[i] += std::max(std::max(xc[k], xc[k+1]), xc[k-1]);// > 0.5*xc[ip]);
      }
    }
    std::cout << "xc= " << xc << std::endl;
    std::cout << "y= " << y << std::endl;
    iterator im = std::max_element(y.begin(), y.end());
    const float period = float(ip)/float(1+std::distance(y.begin(), im));
    return period;
  }

  bool check_signal(const vec_type& xc, float period) {
    // check
    const int m = std::floor(xc.size()/period);
    float sum[2] = { 0, 0 };
    int counter=0;
    for (int i=0; i<m; ++i) {
      const int idx_max = int((i+1.0)*period + 0.5);
      const int idx_min = int((i+0.0)*period + 0.5);
      if (idx_max >= xc.size())
        continue;

      const_iterator i_min = std::min_element(xc.begin()+idx_min,
                                              xc.begin()+idx_max);
      sum[0] += *i_min;
      sum[1] += xc[idx_max];
      std::cout << "sum: " << period << " " << i << " "
                << *i_min << " " << xc[idx_max]
                << std::endl;
      ++counter;
    }
    if (counter) {
      sum[0] /= counter;
      sum[1] /= counter;
    }
    std::cout << "sum: " << sum[0] << " " << sum[1] << std::endl;
    return (sum[0] < 0 &&
            sum[1] > 0 &&
            sum[1]-sum[0] > 0.2);
  }

  float compute_period_precise(const vec_type& xc, float period) const {
    const int m = int(xc.size()/period + 0.5);
    vec_type ps(6*m+1, 0);
    for (int i=-3*m; i<=3*m; ++i)
      ps[3*m+i] = int(period+0.5) + float(i)/float(2*m);

    vec_type y(ps.size(), 0);
    for (int i=0, n=y.size(); i<n; ++i) {
      y[i] = 0.0f;
      int counter=0;
      for (int j=1; j<=m; ++j) {
        const int idx = std::floor(j*ps[i]);
        if (idx+1 >= xc.size())
          continue;
        const float x = j*ps[i]-idx;
        y[i] += xc[idx] + x*(xc[idx+1] - xc[idx]);
        ++counter;
      }
      y[i] = (counter ? y[i]/counter : -100.0f);
      std::cout << "compute_period_precise: " << i << " " << ps[i] << " " << y[i] << " counter=" << counter<< std::endl;
    }
    iterator im = std::max_element(y.begin(), y.end());
    return ps[std::distance(y.begin(), im)];
  }

  float xcorr(const vec_type& v0,
              const vec_type& v1) const {
    float_t s00(0),s01(0),s11(0);
    for (int i=0, n=v0.size(); i<n; ++i) {
      s00 += v0[i]*v0[i];
      s01 += v0[i]*v1[i];
      s11 += v1[i]*v1[i];
    }
    return s01/sqrt(s00*s11);
  }
  vec_type corr(const_iterator i0,
                const_iterator i1) const {
    int m = std::distance(i0, i1)/2;
    vec_type xc(m, 0);
    vec_type yc(m, 0);
    for (int i=0; i<m; ++i) {
      xc[i] = yc[i] = 0.0f;
      for (int j=0; j<m; ++j) {
        xc[i] += i0[  j] * i0[i+j];
        yc[i] += i0[i+j] * i0[i+j];
      }
    }
    const float norm = std::sqrt(xc[0]);
    for (int i=0; i<m; ++i) {
      xc[i] /= (norm*std::sqrt(yc[i]));
      if (std::abs(xc[i]) > 1)
        std::cout << "XXX " << xc[i] << std::endl;
    }
    std::cout << "TEST: " << xc[0] << std::endl;
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
  void normalize(const_iterator sp_beg,    // original signal
                 iterator       snp_beg,   // normalized signal
                 float_t        pThr[2]) { // avg, min,max level
    // adjust pThr[0], pThr[1] iteratively
    pThr[0] = quantile(sp_beg, sp_beg+nt(), 0.1);
    pThr[1] = quantile(sp_beg, sp_beg+nt(), 0.9);

    // max. 20 iteration
    for (int k=0; k<20; ++k) {
      float pThrNew[2] = { 0, 0 };
      int   npThr[2]   = { 0, 0 };
      float thr = 0.5*(pThr[0]+pThr[1]);
      const_iterator isp = sp_beg;
      for (int l=0; l<nt(); ++l, ++isp) {
        const bool b(*isp > thr);
        pThrNew[b] += *isp;
        npThr[b]   += 1;
      }
      pThrNew[0] /= npThr[0];
      pThrNew[1] /= npThr[1];
      std::cout << "normalize k=" << k << " s0,s1=" << pThrNew[0] << " " << pThrNew[1] << " thr=" << thr << std::endl;
      if (std::abs(pThrNew[0]-pThr[0]) < 0.1 &&
          std::abs(pThrNew[1]-pThr[1]) < 0.1) {
        break;
      }
      pThr[0] = pThrNew[0];
      pThr[1] = pThrNew[1];
    }
    // make up normalized signal, clamped to +-2
    const_iterator isp  = sp_beg;
    iterator isnp = snp_beg;
    for (int l=0; l<nt(); ++l, ++isp, ++isnp) {
      const float &x  = *isp;
      float &xn = *isnp;
      xn = 2*(x - pThr[0])/(pThr[1] - pThr[0]) - 1.0f;
      xn = (xn < -2.0f ? -2.0f : xn);
      xn = (xn > +2.0f ? +2.0f : xn);
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
  db::sqlite3::connection::sptr connection_;
  db::sqlite3::statement        stmtFreq_;
  db::sqlite3::statement        stmtTime_;
  db::sqlite3::statement        stmtSig_;
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

    std::vector<unsigned char> v(125*1000, 0);
    std::vector<double>        s(125*1000, 0);

    typedef db::sqlite3::statement::ptime ptime;

    db::sqlite3::connection::sptr connection = db::sqlite3::connection::make("file:test.db");

    db::sqlite3::statement stmtTime;
    stmtTime.init(connection, "SELECT tMin,tMax FROM SpecInfo;");
    while (stmtTime.step()) {
      const ptime t0 = stmtTime.get_column<ptime>(0);
      const ptime t1 = stmtTime.get_column<ptime>(1);
      const boost::posix_time::time_duration dt(0,2,0); // 2 minutes
      std::cout << "t0= " << t0 << " " << " t1= " << t1 << std::endl;
      for (ptime t=t0; t<t1-dt; t+=dt) {
        db::sqlite3::statement stmtSpec;
        stmtSpec.init(connection,
                      str(boost::format("SELECT t,pMin,pMax,s,fMin,fMax,df "
                                        "FROM SpecData JOIN SpecInfo USING (specId)"
                                        "WHERE t>=\"%s\" and t<\"%s\";")
                          % db::sqlite3::statement::date_time_to_string(t0)
                          % db::sqlite3::statement::date_time_to_string(t0+dt)));
        std::cout << "t= " << t << " " <<  (t+dt) << std::endl;
        spec_buf sb(1e3*fMin_kHz, 1e3*fMax_kHz, 4.0, 16*120, 0.25);

        int counter=0;
        while (stmtSpec.step()) {
          const ptime          time   = stmtSpec.get_column<ptime>(0);
          const double         pMin   = stmtSpec.get_column<double>(1);
          const double         pMax   = stmtSpec.get_column<double>(2);
          const unsigned char* bytes  = reinterpret_cast<const unsigned char*>(stmtSpec.column_blob(3));
          const unsigned int   nBytes = stmtSpec.column_bytes(3);
          const double         fMin   = stmtSpec.get_column<double>(4);
          const double         fMax   = stmtSpec.get_column<double>(5);
          const double         df     = stmtSpec.get_column<double>(6);

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
          // double f1 = 1e3*fMax_kHz;
          int    i0 = int((f0-fMin)/df+0.5);
          // int    i1 = int((f1-fMin)/df+1.5);
          // std::cout << "S ";
          // for (int i=i0; i<i1; ++i)
          //   std::cout << s[i] << " ";
          // std::cout << std::endl;;
          sb.insert(counter++, s.begin()+i0);
        }
        sb.proc(t);
        break;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
