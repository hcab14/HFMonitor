// -*- C++ -*-
// $Id$
#ifndef __perseus_pp_h__
#define __perseus_pp_h__

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <perseus-sdr.h>
#include <stdexcept>
#include <string>


class Perseus : public boost::noncopyable {
public:
  Perseus()
    : numPerseus_(perseus_init()) {}

  ~Perseus() {
    perseus_exit();
  }

  unsigned numPerseus() const { return numPerseus_; }
  
  class Receiver : public boost::noncopyable {
  public:
    Receiver() 
      : descr_(0)
      , sampleRate_(0)
      , ddsCenterFrequency_(0)
      , attenId_(0)
      , enablePresel_(1)
      , enablePreamp_(0)
      , enableDither_(0) {}
    
    Receiver(unsigned num)
      : descr_(perseus_open(num))
      , sampleRate_(0)
      , ddsCenterFrequency_(0) 
      , attenId_(0)
      , enablePresel_(1)
      , enablePreamp_(0)
      , enableDither_(0) {
      if (descr_ == 0) 
	throw std::runtime_error("Receiver::Receiver");
    }
    
    double   sampleRate() const { return sampleRate_; }
    double   ddcCenterFrequency() const { return ddsCenterFrequency_; }
    unsigned attenId() const { return attenId_; }
    bool     enablePreamp() const { return enablePreamp_; }
    bool     enableDither() const { return enableDither_; }
    bool     enablePresel() const { return enablePresel_; }

    void downloadFirmware() {
      if (perseus_firmware_download(descr_, NULL) < 0)
	throw std::runtime_error("perseus_firmware_download: " + std::string(perseus_errorstr()));
    }
    
    eeprom_prodid getProductId() {
      eeprom_prodid prodid;
      if (perseus_get_product_id(descr_, &prodid) < 0)
	throw std::runtime_error("perseus_get_product_id: " + std::string(perseus_errorstr()));
      return prodid;
    }

    void fpgaConfig(double sampleRate, std::string fileName) {
      sampleRate_= sampleRate;
      if (perseus_fpga_config(descr_, const_cast<char*>(fileName.c_str())) < 0)
	throw std::runtime_error("perseus_fpga_config: " + std::string(perseus_errorstr()));
    }

    void setAttenuator(int attenId) {
      if (perseus_set_attenuator(descr_, attenId_=attenId) < 0)
	throw std::runtime_error("perseus_set_attenuator: " + std::string(perseus_errorstr()));
    }

    void setADC(bool enableDither, bool enablePreamp) {
      if (perseus_set_adc(descr_, enableDither_=enableDither, enablePreamp_=enablePreamp) < 0)
	throw std::runtime_error("perseus_set_adc: " + std::string(perseus_errorstr()));
    }

    void setDdcCenterFreq(double freq, bool enablePresel) {      
      if (perseus_set_ddc_center_freq(descr_, ddsCenterFrequency_=freq, enablePresel_=enablePresel) < 0)
	throw std::runtime_error("perseus_set_ddc_center_freq: " + std::string(perseus_errorstr()));
    }

    void startAsyncInput(uint32_t buffersize, perseus_input_callback callback, void *cb_extr) {
      if (perseus_start_async_input(descr_, buffersize, callback, cb_extr) < 0)
 	throw std::runtime_error("perseus_start_async_input: " + std::string(perseus_errorstr()));
    }

    void stopAsyncInput() {
      if (perseus_stop_async_input(descr_) < 0)
	throw std::runtime_error("perseus_stop_async_input: " + std::string(perseus_errorstr()));
    }

    ~Receiver() {
      if (descr_) 
	perseus_close(descr_); 
    }
  private:
    perseus_descr*  descr_;
    double          sampleRate_;
    double          ddsCenterFrequency_;
    unsigned        attenId_;
    bool            enablePresel_;
    bool            enablePreamp_;
    bool            enableDither_;
  } ;
  
  typedef boost::shared_ptr<Receiver> ReceiverPtr;
    
  ReceiverPtr getReceiverPtr(unsigned num) {
    if (num < numPerseus()) 
      return ReceiverPtr(new Receiver(num));
    throw std::runtime_error("Perseus::getReceiverPtr");
  }
  
private:
  const unsigned numPerseus_;
} ;

#endif
