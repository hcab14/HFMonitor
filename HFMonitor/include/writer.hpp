// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _WRITER_HPP_cm130110_
#define _WRITER_HPP_cm130110_

#include "gen_filename.hpp"
#include "processor.hpp"

class writer_txt : public processor::base, public gen_filename {
public:
  typedef boost::shared_ptr<writer_txt> sptr;

  writer_txt(const boost::property_tree::ptree& config)
    : base(config)
    , base_path_(config.get<std::string>("<xmlattr>.filePath"))
    , file_period_(gen_filename::str2period(config.get<std::string>("<xmlattr>.filePeriod")))
    , pos_(0) {
    std::cout << "writer_txt::writer_txt" << std::endl;
  }

  virtual ~writer_txt() {}

  // gen_filename methods
  virtual file_period filePeriod()    const { return file_period_; }
  virtual std::string fileExtension() const { return "txt"; }

  virtual void process(processor::base::service::sptr service,
		       processor::base::const_iterator i0, 
		       processor::base::const_iterator i1) {
    std::cout << "writer_txt::process " << service->approx_ptime() << " "
	      << std::distance(i0, i1) << std::endl;
    const boost::filesystem::path
      filepath(gen_file_path(base_path_, service->stream_name(), service->approx_ptime()));

    if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
      std::cerr << "file '" << filepath << "' exists and will be overwritten" << std::endl;
      boost::filesystem::remove(filepath);
    }
    if (not boost::filesystem::exists(filepath)) {
      std::cerr << "new file " << filepath << std::endl;
      std::ofstream ofs(filepath.c_str(), std::ios::binary);        
      // writeT(ofs, header_.change_sample_rate(service->sample_rate_hz()));
      pos_ = ofs.tellp();
    }
    // write data
    std::ofstream ofs(filepath.c_str(), std::ios::in | std::ios::out | std::ios::binary);
    ofs.seekp(pos_);
    std::copy(i0, i1, std::ostream_iterator<char>(ofs));
    ofs << "\n";
    pos_ = ofs.tellp();      
  }
  
protected:
private:
  std::string    base_path_;
  gen_filename::file_period file_period_;
  std::streampos pos_;    
} ;

#endif // _WRITER_HPP_cm130110_
