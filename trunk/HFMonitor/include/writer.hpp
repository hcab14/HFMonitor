// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
#ifndef _WRITER_TXT_HPP_cm130110_
#define _WRITER_TXT_HPP_cm130110_

#include <boost/date_time/posix_time/posix_time.hpp>

#include "gen_filename.hpp"
#include "processor.hpp"

class writer_txt : public processor::base, public gen_filename {
public:
  typedef boost::shared_ptr<writer_txt> sptr;

  writer_txt(const boost::property_tree::ptree& config)
    : base(config)
    , base_path_(config.get<std::string>("<xmlattr>.filePath"))
    , file_period_(gen_filename::str2period(config.get<std::string>("<xmlattr>.filePeriod")))
    , time_format_(config.get<std::string>("<xmlattr>.timeFormat", "%Y-%m-%d %H:%M:%s"))
    , pos_(0) {}

  virtual ~writer_txt() {}

  // gen_filename methods
  virtual file_period filePeriod()    const { return file_period_; }
  virtual std::string fileExtension() const { return "txt"; }

  virtual void process(service::sptr sp,
		       const_iterator i0,
		       const_iterator i1) {
    const boost::filesystem::path
      filepath(gen_file_path(base_path_, sp->stream_name(), sp->approx_ptime()));

    if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
      LOG_ERROR(str(boost::format("file '%s' exists and will be overwritten") % filepath));
      boost::filesystem::remove(filepath);
    }
    if (not boost::filesystem::exists(filepath)) {
      LOG_INFO(str(boost::format("creating new file '%s'") % filepath));
      std::ofstream ofs(filepath.c_str(), std::ios::binary);
      pos_ = ofs.tellp();
    }
    // write data
    std::ofstream ofs(filepath.c_str(), std::ios::in | std::ios::out | std::ios::binary);
    ofs.seekp(pos_);
    ofs << make_time_label(sp->approx_ptime()) << " ";
    std::copy(i0, i1, std::ostream_iterator<char>(ofs));
    ofs << "\n";
    pos_ = ofs.tellp();
  }

protected:
  std::string make_time_label(ptime t) const {
    std::stringstream oss;
    oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet(time_format_.c_str())));
    oss << t;
    return oss.str();
  }
private:
  const std::string               base_path_;
  const gen_filename::file_period file_period_;
  const std::string               time_format_;
  std::streampos pos_;
} ;

#endif // _WRITER_TXT_HPP_cm130110_
