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
#ifndef _mat_spectrum_saver_hpp_cm110902_
#define _mat_spectrum_saver_hpp_cm110902_
#include <fstream>
#include <sstream>

#include <octave/oct.h>
#include <octave/Matrix.h>
#include <octave/ov.h>
#include <octave/load-save.h>
#include <octave/ls-mat5.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#include "util.hpp"
#include "Spectrum.hpp"
#include "logging.hpp"

class mat_spectrum_saver {
public:
  typedef boost::property_tree::ptree ptree;
  typedef boost::posix_time::ptime ptime;

  mat_spectrum_saver(const ptree& config_proc, 
                     const ptree& config_saver)
    : name_(config_proc.get<std::string>("<xmlattr>.label"))
    , byte_limit_(1024*get_opt_def<size_t>(config_proc, config_saver, "<xmlattr>.maxBufferSize_kB", 500))
    , base_path_(get_opt_def<std::string>(config_proc, config_saver, "<xmlattr>.dataPath", "Data/"))
    , minutes_(std::min(size_t(60), 
                        get_opt_def<size_t>(config_proc, config_saver, "<xmlattr>.minutesPerFile", 10)))
    , compress_(false)
    , save_as_floats_(false)
    , mat7_format_(false)
    , global_(false)
    , pos_(0)
    , counter_(0)
    , current_data_size_(0) {}
  
  template<typename fft_type>
  void save_spectrum(const frequency_vector<fft_type>& spec, ptime t) {
    const boost::filesystem::path filepath(gen_filepath(t));
    if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
      LOG_WARNING(str(boost::format("file '%s' exists and will be overwritten") % filepath));
      boost::filesystem::remove(filepath);        
    }
    if (not boost::filesystem::exists(filepath)) {
      boost::filesystem::ofstream ofs(filepath, std::ios::binary);
      write_header(ofs, load_save_format(LS_MAT5_BINARY));
      freq_ = Matrix(1, spec.size());
      spec_ = int16NDArray(dim_vector(0, spec.size()));
      time_ = Matrix(0, 1);
      for (size_t i=0; i<spec.size(); ++i)
        freq_.elem(0, i) = spec[i].first;
      counter_ = current_data_size_ = 0 ;
      save_mat_var(ofs, freq_, "freq_Hz");
      pos_ = ofs.tellp(); 
      save_mat_var(ofs, spec_, "spec"+varname_suffix());
      save_mat_var(ofs, time_, "time"+varname_suffix());
    }
    int16NDArray old_spec(spec_);
    Matrix       old_time(time_);
    size_t n(old_spec.rows()); 

    if (current_data_size_ > byte_limit_) {
      n = 0;
      current_data_size_ = 0;
      counter_++;
      boost::filesystem::ifstream ifs(filepath, std::ios::binary);
      ifs.seekg(0, std::ios::end);
      pos_ = ifs.tellg();
      old_spec.resize(dim_vector(0, spec.size()));
      old_time.resize(0, 1);
    }
    spec_.resize(dim_vector(n+1, spec.size()));
    time_.resize(n+1, 1);

    spec_.insert(old_spec, 0, 0);
    time_.insert(old_time, 0, 0);
    for (size_t i=0; i<spec.size(); ++i)
      spec_.elem(n, i) = boost::int16_t(100 * 20.*std::log10(spec[i].second) + 0.5);
    time_.elem(n, 0) = ptime_to_datenum(t);

    // overwrite old variables with new ones
    boost::filesystem::fstream ofs(filepath, std::ios::binary | std::ios::in | std::ios::out);
    ofs.seekp(pos_);
    save_mat_var(ofs, spec_, "spec_"+varname_suffix());
    save_mat_var(ofs, time_, "time_"+varname_suffix());
    current_data_size_ = ofs.tellp() - pos_;
  }
protected:
  bool save_mat_var(std::ostream& os,
                    const octave_value& ov,
                    std::string varname) const {
    return save_mat5_binary_element(os, ov, varname, global_, mat7_format_, save_as_floats_, compress_);
  }
  boost::filesystem::path gen_filepath(ptime t) const {
    boost::filesystem::path p(base_path_+"/"+name_);
    boost::filesystem::create_directories(p);

    std::stringstream oss;
    oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("y%Y-m%m-d%d_H%HM%M")));
    const boost::posix_time::time_duration td(t.time_of_day());         
    t= boost::posix_time::ptime(t.date(), 
                                boost::posix_time::time_duration(td.hours(), 
                                                                 minutes_*(td.minutes()/minutes_), 0));
    oss << t << ".mat";
    return p/=(oss.str());
  }
  std::string varname_suffix() const {
    return str(boost::format("%04X") % counter_);
  }
  static double ptime_to_datenum(const ptime& p) {
    using namespace boost::gregorian;
    using namespace boost::posix_time;;
    const time_duration dt(p - ptime(date(1970, Jan, 1)));
    return 719529.+double(dt.ticks())/(60.*60.*24.*time_duration::ticks_per_second());
  }
  template<typename T>
  static
  T get_opt_def(const ptree& p1, const ptree& p2, std::string name, T def) {
    return p1.get<T>(name, p2.get<T>(name, def));
  }
private:
  const std::string name_;
  const size_t      byte_limit_;
  const std::string base_path_;
  const size_t      minutes_;
  const bool        compress_;
  const bool        save_as_floats_;
  const bool        mat7_format_;
  bool              global_;
  std::streampos    pos_;
  size_t            counter_;
  size_t            current_data_size_;
  Matrix            freq_;
  int16NDArray      spec_;
  Matrix            time_;  
} ;

#endif // _mat_spectrum_saver_hpp_cm110902_
