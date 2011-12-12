// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef __perseus_pp_h__
#define __perseus_pp_h__

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <perseus-sdr.h>
#include <stdexcept>
#include <string>

#include "logging.hpp"

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
      ASSERT_THROW(descr_ != 0);
    }
    
    double   sampleRate() const { return sampleRate_; }
    double   ddcCenterFrequency() const { return ddsCenterFrequency_; }
    unsigned attenId() const { return attenId_; }
    bool     enablePreamp() const { return enablePreamp_; }
    bool     enableDither() const { return enableDither_; }
    bool     enablePresel() const { return enablePresel_; }

    void downloadFirmware() {
      ASSERT_THROW_PERSEUS(perseus_firmware_download(descr_, NULL) == 0);
    }
    
    eeprom_prodid getProductId() {
      eeprom_prodid prodid;
      ASSERT_THROW_PERSEUS(perseus_get_product_id(descr_, &prodid) == 0);
      return prodid;
    }

    void fpgaConfig(int sampleRate) {
      sampleRate_= sampleRate;
      ASSERT_THROW_PERSEUS(perseus_set_sampling_rate(descr_, sampleRate) == 0);
    }

    void setAttenuator(int attenId) {
      ASSERT_THROW_PERSEUS(perseus_set_attenuator(descr_, attenId_=attenId) == 0);
    }

    void setADC(bool enableDither, bool enablePreamp) {
      ASSERT_THROW_PERSEUS(perseus_set_adc(descr_, 
                                           enableDither_=enableDither, 
                                           enablePreamp_=enablePreamp) == 0);
    }

    void setDdcCenterFreq(double freq, bool enablePresel) {      
      ASSERT_THROW_PERSEUS(perseus_set_ddc_center_freq(descr_, 
                                                       ddsCenterFrequency_=freq, 
                                                       enablePresel_=enablePresel) == 0);
    }

    void startAsyncInput(uint32_t buffersize, perseus_input_callback callback, void *cb_extr) {
      ASSERT_THROW_PERSEUS(perseus_start_async_input(descr_, buffersize, callback, cb_extr) == 0);
    }

    void stopAsyncInput() {
      ASSERT_THROW_PERSEUS(perseus_stop_async_input(descr_) == 0);
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
    ASSERT_THROW(num < numPerseus());
    return ReceiverPtr(new Receiver(num));
  }
  
private:
  const unsigned numPerseus_;
} ;

#endif
