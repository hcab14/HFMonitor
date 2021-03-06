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
#ifndef _WRITER_TXT_HPP_cm130110_
#define _WRITER_TXT_HPP_cm130110_

#include <boost/date_time/posix_time/posix_time.hpp>

#include "gen_filename.hpp"
#include "processor.hpp"

/// base class for writing into text files
class writer_txt_base : public gen_filename {
public:
  typedef boost::shared_ptr<writer_txt_base> sptr;
  typedef boost::posix_time::ptime ptime;

  writer_txt_base(const boost::property_tree::ptree& config)
    : base_path_(config.get<std::string>("<xmlattr>.filePath"))
    , file_period_(gen_filename::str2period(config.get<std::string>("<xmlattr>.filePeriod")))
    , time_format_(config.get<std::string>("<xmlattr>.timeFormat", "%Y-%m-%d %H:%M:%s"))
    , pos_(0) {}

  virtual ~writer_txt_base() {}

  // gen_filename methods
  virtual file_period filePeriod()    const { return file_period_; }
  virtual std::string fileExtension() const { return "txt"; }

  /// dumps \c rp into a file
  processor::result_base::sptr dump_result(processor::result_base::sptr rp,
                                           std::string extra_header) {
    if (!rp)
      return rp;
    if (rp->format() != "TXT_0000") {
      LOG_ERROR(str(boost::format("writer_txt::dump_result: unknown format '%s'") % rp->format()));
      return rp;
    }

    const boost::filesystem::path
      filepath(gen_file_path(base_path_, rp->name(), rp->approx_ptime()));

//     if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
//       LOG_ERROR(str(boost::format("file '%s' exists and will be overwritten") % filepath));
//       boost::filesystem::remove(filepath);
//     }
    if (not boost::filesystem::exists(filepath)) {
      LOG_INFO(str(boost::format("creating new file '%s'") % filepath));
      std::ofstream ofs(filepath.c_str(), std::ios::binary);
      ofs << extra_header;
      rp->dump_header(ofs) << "\n";
      pos_ = ofs.tellp();
    } else {
      std::ifstream ifs(filepath.c_str(), std::ios::binary);
      ifs.seekg(0, ifs.end);
      pos_ = ifs.tellg();
    }
    // write data
    std::ofstream ofs(filepath.c_str(), std::ios::in | std::ios::out | std::ios::binary);
    ofs.seekp(pos_);
    ofs << make_time_label(rp->approx_ptime()) << " ";
    rp->dump_data(ofs) << "\n";
    pos_ = ofs.tellp();
    return rp;
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

/// processor for writing results (@ref processor::result_base) into text files
class writer_txt : public processor::base, public writer_txt_base {
public:
  typedef boost::shared_ptr<writer_txt> sptr;

  class result : public processor::result_base {
  public:
    typedef boost::shared_ptr<result> sptr;

    virtual ~result() {}

    static sptr make(std::string name,
                     ptime t,
                     std::string header,
                     std::string data) {
      return sptr(new result(name, t, header, data));
    }
    virtual std::ostream& dump_header(std::ostream& os) const { return os << header_; }
    virtual std::ostream& dump_data(std::ostream& os)   const { return os << data_; }

  protected:
  private:
    result(std::string name,
           ptime t,
           std::string header,
           std::string data)
      : result_base(name, t)
      , header_(header)
      , data_(data) {}

    const std::string header_;
    const std::string data_;
  } ;

  writer_txt(const boost::property_tree::ptree& config)
    : base(config)
    , writer_txt_base(config) {}

  virtual ~writer_txt() {}

  /// process method
  virtual void process(service::sptr sp,
		       const_iterator i0,
		       const_iterator i1) {
    // when empty input: NOP
    if ( i0 == i1) return;
    // save header lines (which start with "#")
    if (*i0 == '#') {
      header_.clear();
      header_.resize(std::distance(i0, i1));
      std::copy(i0, i1, header_.begin());
      return;
    }
    // else: call @ref dump_result
    std::string data(std::distance(i0, i1), ' ');
    std::copy(i0, i1, data.begin());
    dump_result(result::make(sp->stream_name(), sp->approx_ptime(), header_, data),
                ""); // no extra header: extra headers are supposed to be contained in header_
  }

protected:
private:
  std::string    header_;
} ;

#endif // _WRITER_TXT_HPP_cm130110_
