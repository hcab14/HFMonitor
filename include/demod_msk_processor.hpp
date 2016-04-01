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
#ifndef _DEMOD_MSK_PROCESSOR_HPP_cm130214_
#define _DEMOD_MSK_PROCESSOR_HPP_cm130214_

#include <cmath>
#include <complex>
#include <iostream>
#include <sstream>
#include <vector>
#include <bitset>

#include <boost/property_tree/ptree.hpp>

#include "demod/msk.hpp"
#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "filter/fir_filter.hpp"
#include "logging.hpp"
#include "processor.hpp"
#include "Spectrum.hpp"
#include "aligned_vector.hpp"

#include <complex>
#include <vector>

/*! \addtogroup processors
 *  @{
 * \addtogroup demod_msk demod_msk
 * MSK demodulation
 * 
 * @{
 */

class rtcm_decoder {
public:
  typedef std::vector<bool> bit_vector_type;
  typedef bit_vector_type::iterator iterator;
  typedef bit_vector_type::const_iterator const_iterator;

  rtcm_decoder()
    : state_(-3)
    , bit_counter_(0)
    , n_data_words_(0)
    , frame_()
    , msg_type_(0)
    , stn_num_(0)
    , z_count_(0)
    , seq_(-1)
    , num_(0) {}

  union frame {
    frame()
      : data(0) {}

    boost::uint32_t data;
    struct {
      boost::uint32_t parity  :  6;
      boost::uint32_t payload : 24;
      boost::uint32_t b30     :  1;
      boost::uint32_t b29     :  1;
    } f;

    boost::uint8_t preamble() const {
      return ((f.payload>>16) & 0xFF);
    }

    bool check_preamble() const {
      return (preamble() == 0x66 || preamble() == 0x99);        
    }
    
    bool check_parity(boost::uint32_t &d) const {
      std::bitset<30> br(data);
      
      std::bitset<30> brr;
      std::bitset<30> bd;
      for (int i=0; i<30; ++i)
        brr[29-i] = ((data>>i) & 1);
#if 0      
      std::cout << "=== check: data= " << std::bitset<32>(data) << std::endl;
      std::cout << "=== check: payload= " << std::bitset<24>(f.payload) << std::endl;
      std::cout << "=== check: parity= " << std::bitset<6>(f.parity) << std::endl;
      std::cout << "=== check: b2930= " << f.b29 << f.b30 << " " << brr[28] << brr[29] <<  std::endl;
      std::cout << "=== check: " << ((f.payload>>16) &0xFF) << " " << ((data>>22)&0xFF) << std::endl;
      std::cout << "=== check: " << f.payload << std::endl;
      std::cout << "=== check: brr= " << brr << std::endl;
#endif
      d = 0;
      for (int i=0; i<24; ++i) {
        bd[i] = (f.b30 ^ brr[i]);
        d    |= (bd[i] << (23-i));
        // d    |= (bd[i] << (i));
      }
      
      bd[24] = f.b29 ^ bd[0] ^ bd[1] ^ bd[2] ^ bd[4] ^ bd[5] ^ bd[ 9] ^ bd[10] ^ bd[11] ^ bd[12] ^ bd[13] ^ bd[16] ^ bd[17] ^ bd[19] ^ bd[22];
      bd[25] = f.b30 ^ bd[1] ^ bd[2] ^ bd[3] ^ bd[5] ^ bd[6] ^ bd[10] ^ bd[11] ^ bd[12] ^ bd[13] ^ bd[14] ^ bd[17] ^ bd[18] ^ bd[20] ^ bd[23];
      bd[26] = f.b29 ^ bd[0] ^ bd[2] ^ bd[3] ^ bd[4] ^ bd[6] ^ bd[ 7] ^ bd[11] ^ bd[12] ^ bd[13] ^ bd[14] ^ bd[15] ^ bd[18] ^ bd[19] ^ bd[21];
      bd[27] = f.b30 ^ bd[1] ^ bd[3] ^ bd[4] ^ bd[5] ^ bd[7] ^ bd[ 8] ^ bd[12] ^ bd[13] ^ bd[14] ^ bd[15] ^ bd[16] ^ bd[19] ^ bd[20] ^ bd[22];
      bd[28] = f.b30 ^ bd[0] ^ bd[2] ^ bd[4] ^ bd[5] ^ bd[6] ^ bd[ 8] ^ bd[ 9] ^ bd[13] ^ bd[14] ^ bd[15] ^ bd[16] ^ bd[17] ^ bd[20] ^ bd[21] ^ bd[23];
      bd[29] = f.b29 ^ bd[2] ^ bd[4] ^ bd[5] ^ bd[7] ^ bd[8] ^ bd[ 9] ^ bd[10] ^ bd[12] ^ bd[14] ^ bd[18] ^ bd[21] ^ bd[22] ^ bd[23];
      
      // std::cout << "=== check: bd= " << bd << std::endl;

      int sum=0;
      for (int i=24; i<30; ++i)
        sum += (brr[i]==bd[i]);
      // std::cout << "=== check: sum=" << sum << std::endl;
      return (sum==6);
    }
  } ;

  void decode(bool b) {
    ++bit_counter_;
    frame_.data  = (frame_.data << 1);
    frame_.data |= b;

    // std::cout << "D *** data " << std::bitset<32>(frame_.data) << std::endl;
    boost::uint32_t data = 0;
    switch (state_) {
    case -3:
      if (frame_.check_preamble()) {
        std::cout << "D *** found preamble ***" << std::endl;
        state_ = -2;
        bit_counter_ = 0;
      }
      break;
    case -2:
      if (bit_counter_ == 30) {
        if (frame_.check_parity(data)) {
          state_       = -1;
          bit_counter_ =  0;
          msg_type_ = ((data>>10) & 0x3F);
          stn_num_  = (data & 0x3FF);
          std::cout << "D *** H1 type=" << int(msg_type_) << " stn=" << stn_num_ << std::endl;
          if (stn_num_ == 0) state_ = -3;
        } else {
          state_ = -3;
        }
      }
      break;
    case -1:
      if (bit_counter_ == 30) {
        if (frame_.check_parity(data)) {
          state_ = 0;
          z_count_ = ((data>>11) & 0x1FFF);
          int seq_new = ((data>> 8) & 0x7);
          num_     = ((data>> 3) & 0x1F);
          std::cout << "D *** H2 z_count=" << z_count_ << " seq=" << int(seq_) << " num_frames=" << int(num_) << std::endl;
          if (num_ == 0)
            state_ = -3;
          // if (seq_ != -1 && seq_new != ((seq_+1)%8)) {
          //   state_ = -3;
          //   seq_ = -1;
          // } else {
            seq_ = seq_new;
          // }
          n_data_words_ = num_;
          data_.clear();
        } else {
          state_ = -3;
        }
        bit_counter_ = 0;
      }
      break;
    default:
      if (bit_counter_ == 30) {
        if (frame_.check_parity(data)) {
          
          data_.push_back( data     &0xFF);
          data_.push_back((data>> 8)&0xFF);
          data_.push_back((data>>16)&0xFF);

          ++state_;
          std::cout << "D *** msg #" << state_ << std::endl;;
          if (state_ == n_data_words_) {
            decode_message();
            state_ = -2;
          }
        } else {
          state_ = -3;
        } 
        bit_counter_ = 0;
      }
    }
  }

  struct __attribute__((__packed__)) msg_1  {
    int sf1() const { return sf1_; }
    int sf2() const { return sf2_; }
    int sf3() const { return sf3_; }

    int udre1() const { return udre1_; }
    int udre2() const { return udre2_; }
    int udre3() const { return udre3_; }

    int sat_id1() const { return sat_id1_; }
    int sat_id2() const { return sat_id2_; }
    int sat_id3() const { return sat_id3_; }

    double prc1() const { double f[2] = {0.02,0.32}; return f[sf1()]*conv(prc1_, 16); }
    double prc2() const { double f[2] = {0.02,0.32}; return f[sf2()]*conv(prc2_, 16); }
    double prc3() const { double f[2] = {0.02,0.32}; return f[sf3()]*conv((prc3_up_<<8)+prc3_low_, 16); }

    double rrc1() const { double f[2] = {0.002, 0.032}; return f[sf1()]*conv(rrc1_, 8); }
    double rrc2() const { double f[2] = {0.002, 0.032}; return f[sf2()]*conv(rrc2_, 8); }
    double rrc3() const { double f[2] = {0.002, 0.032}; return f[sf3()]*conv(rrc3_, 8); }

    int iod1() const { return iod1_; }
    int iod2() const { return iod2_; }
    int iod3() const { return iod3_; }

    size_t prc1_     : 16;
    size_t sat_id1_  :  5;
    size_t udre1_    :  2;
    size_t sf1_      :  1;

    size_t sat_id2_  :  5;
    size_t udre2_    :  2;
    size_t sf2_      :  1;
    size_t iod1_     :  8;
    size_t rrc1_     :  8;

    size_t rrc2_     :  8;
    size_t prc2_     : 16; 

    size_t prc3_up_  :  8;
    size_t sat_id3_  :  5;
    size_t udre3_    :  2;
    size_t sf3_      :  1;
    size_t iod2_     :  8;

    size_t iod3_     :  8;
    size_t rrc3_     :  8;
    size_t prc3_low_ :  8;

    static boost::int32_t conv(boost::uint64_t x, int bits) {
      const boost::uint64_t pow2 = (1<<(bits-1));
      return ((x & pow2)
              ? (-1 - ((x-pow2) ^ (pow2-1)))
              : x);
    }

  } ;
  struct __attribute__((__packed__)) msg_7 {
    double lat() const {
      return 0.002747*conv(lat_, 16);
    }
    double lon() const {
      return 0.005493*conv((lon_h_<<8)+lon_l_, 16);
    }
    double range() const {
      return 1.*range_;
    }
    double freq() const {
      return 190.0 + 0.1*double((freq_h_<<6)+freq_l_);
    }
    boost::uint32_t bcid() const {
      return bcid_;
    }

    static boost::int32_t conv(boost::uint64_t x, int bits) {
      const boost::uint64_t pow2 = (1<<(bits-1));
      return ((x & pow2)
              ? (-1 - ((x-pow2) ^ (pow2-1)))
              : x);
    }
    size_t lon_h_     :  8;
    size_t lat_       : 16;

    size_t freq_h_    :  6;
    size_t range_     : 10;
    size_t lon_l_     :  8;

    size_t  bc_coding_ :  1;
    size_t  sync_type_ :  1;
    size_t  mod_code_  :  1;
    size_t  bitrate_   :  3;

    size_t bcid_      : 10;
    size_t health_    :  2;
    size_t freq_l_    :  6;
  } ;

  void decode_message() const {
    switch (msg_type_) {
    case 7: {
      for (size_t i=0; i<data_.size(); i+=9) {
        const msg_7 *m = (const msg_7*)(&data_[i]);
        std::cout << "D*** MSG_7:" 
                  << m->lat() << " "
                  << m->lon() << " "
                  << m->range() << " "
                  << m->freq()  << " "
                  << m->bcid() << " | ";
        for (int j=0; j<9; ++j)
          std::cout << int(data_[i+j]) << " ";
        std::cout << std::endl;
      }
      break;
      case 1:
      case 9: {
        for (size_t i=0; i<data_.size(); i+=15) {
          const msg_1 *m = (const msg_1*)(&data_[i]);
          std::cout << "D*** MSG_1:"
                    << m->sat_id1() << " "
                    << m->sf1() << " "
                    << m->udre1() << " "
                    << m->prc1() << " "
                    << m->rrc1() << " "
                    << m->iod1() << " "
                    << std::endl;
          std::cout << "D*** MSG_1:"
                    << m->sat_id2() << " "
                    << m->sf2() << " "
                    << m->udre2() << " "
                    << m->prc2() << " "
                    << m->rrc2() << " "
                    << m->iod2() << " "
                    << std::endl;
          std::cout << "D*** MSG_1:"
                    << m->sat_id3() << " "
                    << m->sf3() << " "
                    << m->udre3() << " "
                    << m->prc3() << " "
                    << m->rrc3() << " "
                    << m->iod3() << " "
                    << std::endl;
        }
      }
        break;
    }
    default:
      ; // NOP
    }
  }
protected:
private:
  int   state_;
  int   bit_counter_;
  int   n_data_words_;
  frame frame_;
  boost::uint8_t  msg_type_;
  boost::uint16_t stn_num_;
  boost::uint32_t z_count_;
  boost::int16_t   seq_;
  boost::uint8_t  num_;
  std::vector<boost::uint8_t> data_;
} ;


/// MSK demodulation processor
class demod_msk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_msk_processor> sptr;
  typedef std::complex<float> complex_type;
  
  class result_phase : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_phase> sptr;
    
    virtual ~result_phase() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fm_Hz,
                     double amplitude,
                     double sn_db,
                     double phase_rad) {
      return sptr(new result_phase(name, t, fc_Hz, fm_Hz, amplitude, sn_db, phase_rad));
    }

    double fc_Hz()     const { return fc_Hz_; }
    double fm_Hz()     const { return fm_Hz_; }
    double amplitude() const { return amplitude_; }
    double sn_db()     const { return sn_db_; }
    double phase_rad() const { return phase_rad_; }

    virtual std::ostream& dump_header(std::ostream& os) const {      
      return os
        << "# fc[Hz] = " << boost::format("%15.8f") % fc_Hz() << "\n"
        << "# fm[Hz] = " << boost::format("%7.3f")  % fm_Hz() << "\n"
        << "# Time_UTC Amplitude[dB] S/N[db] Phase[rad] ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      return os
        << boost::format("%7.2f") % amplitude() << " "
        << boost::format("%7.2f") % sn_db()     << " "
        << boost::format("%6.3f") % phase_rad();
    }
    
  protected:
  private:
    result_phase(std::string name,
                 ptime  t,
                 double fc_Hz,
                 double fm_Hz,
                 double amplitude,
                 double sn_db,
                 double phase_rad)
      : result_base(name, t)
      , fc_Hz_(fc_Hz)
      , fm_Hz_(fm_Hz)
      , amplitude_(amplitude)
      , sn_db_(sn_db)
      , phase_rad_(phase_rad) {}
    
    const double fc_Hz_;
    const double fm_Hz_;
    const double amplitude_;
    const double sn_db_;
    const double phase_rad_;
  } ; 

  class result_bits : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_bits> sptr;
    typedef std::vector<bool> bit_vector_type;
    typedef bit_vector_type::iterator iterator;
    typedef bit_vector_type::const_iterator const_iterator;

    virtual ~result_bits() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fm_Hz) {
      return sptr(new result_bits(name, t, fc_Hz, fm_Hz));
    }

    double fc_Hz()   const { return fc_Hz_; }
    double fm_Hz()   const { return fm_Hz_; }
    double quality() const { return quality_; }

    void  set_quality(double q) { quality_ = q; }

    size_t size() const { return bitvec_.size(); }
    const_iterator begin() const { return bitvec_.begin(); }
    const_iterator end() const { return bitvec_.end(); }

    void clear() { bitvec_.clear(); }
    void push_back(bool bit) { bitvec_.push_back(bit); }

    const bit_vector_type& bits() const { return bitvec_; }

    virtual std::ostream& dump_header(std::ostream& os) const {      
      return os
        << "# fc[Hz] = " << boost::format("%15.8f") % fc_Hz() << "\n"
        << "# fm[Hz] = " << boost::format("%7.3f")  % fm_Hz() << "\n"
        << "# Time_UTC quality[%] bits ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      os << boost::format("%3.0f") % (100.*quality()) << " ";
      for (const_iterator i(begin()); i!=end();) {
        unsigned char x(0);
        for (size_t j(0); j<4 && i!=end(); ++j, ++i)
          x |= (*i << (3-j));
        const char c(x<10 ? '0'+x : 'A'+x-10);
        os << c;
      }
      return os;
    }
    
  protected:
  private:
    result_bits(std::string name,
               ptime  t,
               double fc_Hz,
               double fm_Hz)
      : result_base(name, t)
      , fc_Hz_(fc_Hz)
      , fm_Hz_(fm_Hz)
      , quality_(0) {}
    
    const double    fc_Hz_;
    const double    fm_Hz_;
    double          quality_;
    bit_vector_type bitvec_;
  } ;
 
  typedef boost::posix_time::time_duration time_duration;
  typedef FFT::FFTWTransform<float> fft_type;

  demod_msk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fm_Hz_(config.get<double>("<xmlattr>.fm_Hz"))
    , record_bits_(config.get<bool>("<xmlattr>.recordBits", false))
    , dwl_Hz_(config.get<double>("<xmlattr>.dwl_Hz"))
    , period_Sec_(config.get<double>("<xmlattr>.period_Sec"))
    , min_SN_db_(config.get<double>("<xmlattr>.min_SN_db"))
    , max_offset_ppb_rms_(config.get<double>("<xmlattr>.max_offset_ppb_rms"))
    , downscale_factor_(1)
    , delta_f_Hz_(0)
    , filter_(401, 0.05*fm_Hz_/200.)
    , filter_amp_pm_    (  config.get<double>("<xmlattr>.ampl_lowpass_tc_Sec"), period_Sec_)
    , filter_amp_center_(5*config.get<double>("<xmlattr>.ampl_lowpass_tc_Sec"), period_Sec_)
    , phase_(0)
    , signal_present_(0) {
    filter_plus_.add (Filter::LowPass<frequency_vector<double> >::make(1.0, 300));
    filter_minus_.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 300));
  }
  
  virtual ~demod_msk_processor() {}

  double fc_Hz() const { return fc_Hz_; }
  double fm_Hz() const { return fm_Hz_; }

  double delta_f_Hz() const { return delta_f_Hz_; }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {

    if (sp->offset_ppb_rms() > max_offset_ppb_rms_) {
      LOG_INFO(str(boost::format("offset_ppb = %.2f > %.2f is too big") % sp->offset_ppb_rms() % max_offset_ppb_rms_));
      return;
    }

    // compute downscale factor
    // set up a new filter
    bool is_first_call(demod_msk_ == 0);

    const double offset_ppb(sp->offset_ppb());
    double offset_Hz((fc_Hz() + delta_f_Hz()) * offset_ppb*1e-9);

    // const size_t len=std::distance(i0,i1);
    // std::vector<std::complex<float> > phases(len);
    // for (int i=0; i<len; ++i) {
    //   phases[i] = std::exp(std::complex<float>(0,i*float(2*M_PI)*(fc_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz()));
    // }

    if (not demod_msk_) {
      const double f_shift0_Hz(fc_Hz() - sp->center_frequency_Hz());
      std::cout << "f_shift0_Hz " << f_shift0_Hz << "\n";
      double f_shift_Hz(filter_.shift(f_shift0_Hz/sp->sample_rate_Hz())*sp->sample_rate_Hz());
       //double f_shift_Hz(f_shift0_Hz);
      std::cout << "f_shift_Hz " << f_shift_Hz << "\n";
      std::cout << "fc_Hz " << fc_Hz() << "\n";
      delta_f_Hz_ = sp->center_frequency_Hz() + f_shift_Hz - fc_Hz();
      std::cout << "fc_Hz " << fc_Hz() << " " << delta_f_Hz() << "\n";

      // 0.05 = 250/sp->sample_rate_Hz() for 200 baud
      filter_.design(401, 1.05*fm_Hz_/sp->sample_rate_Hz());

      offset_Hz = (fc_Hz() + delta_f_Hz()) * offset_ppb*1e-9;
      std::cout << "offset_Hz " << offset_Hz << " "
                << fc_Hz() << " " << filter_.get_shift()*sp->sample_rate_Hz() << " " << offset_ppb << "\n";

      const size_t length(std::distance(i0, i1));
      downscale_factor_ = std::max(1, int(0.25*sp->sample_rate_Hz()/(1.25*fm_Hz_)));
      for (; downscale_factor_> 1 && std::fmod(double(length), double(downscale_factor_))!=0.0; --downscale_factor_)
        ;
      // downscale_factor_ = 1;
      std::cout << "downscale_factor= " << downscale_factor_ << std::endl;

      demod_msk_ = demod::msk::make(sp->sample_rate_Hz()/downscale_factor_,
                                    delta_f_Hz() + offset_Hz,
                                    0.5*fm_Hz(), // baud/2
                                    dwl_Hz_, period_Sec_);
      std::cout << "shift: " << fc_Hz() << " " << sp->center_frequency_Hz() << " "
                << f_shift0_Hz << " " << f_shift_Hz
                << std::endl;

      filter_amp_pm_.reset(-1);
      filter_amp_center_.reset(-1);
      signal_present_ = 0;
    }


    const size_t length(std::distance(i0, i1)/downscale_factor_);
    if (length != fftw_.size())
      fftw_.resize(length);

    demod_msk_->update_ppb(offset_ppb,
                           delta_f_Hz() + offset_Hz,
                           0.5*fm_Hz());

    // filter input
    aligned_vector<complex_type> 
      samples(length, complex_type(0)),
      samples2(length, complex_type(0));
    aligned_vector<complex_type>::iterator
      j(samples.begin()),
      k(samples2.begin());
    for (const_iterator i(i0); i!=i1; ++i) {
      // filter_.insert(phases[std::distance(i0, i)]* (*i));
      filter_.insert(*i);
      if (downscale_factor_ == 1 ||
          (std::distance(i0, i) % downscale_factor_) == 0) {
        const complex_type sample(filter_.process());
        *j++ = sample;
        *k++ = sample*sample;
      }
    }
    std::cout << "TEST " << length << " == "
              << std::distance(samples.begin(), j) << " == "
              << std::distance(samples2.begin(), k) 
              << std::endl;

    {
      // fftw_.transformRange(samples.begin(), samples.end(), FFT::WindowFunction::Blackman<float>(samples.size()));
      // const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz()/downscale_factor_, -2.*delta_f_Hz());
      
      // for (int i=0; i<s.size(); ++i)
      //   std::cout << "SPEC: " << " " << s.index2freq(i) << " " << std::abs(s[i]) << std::endl;

      // for (int i=0; i<samples2.size(); ++i)
      //   std::cout << "SIG: " << samples[i].real() << " " << samples[i].imag() << std::endl;

    }

    double baud(0), delta_f(0), sn(0);
    const bool is_signal = (detect_msk(sp, samples2.begin(), samples2.end(), baud, delta_f, sn) &&
                            sn > min_SN_db_       &&
                            std::abs(delta_f) < 5 &&
                            (std::abs(baud-100) < 5 ||
                             std::abs(baud-200) < 5));
    
    for (const_iterator i(samples.begin()); i!=samples.end(); ++i) {
      const complex_type sample(*i);
      demod_msk_->process(sample);
      
      // amplitude and phase data
      if (not is_first_call && demod_msk_->phase_available()) {
        const double amplitude_minus (10*std::log10(std::abs(demod_msk_->gf_minus().x())));
        const double amplitude_center(10*std::log10(std::abs(demod_msk_->gf_center().x())));
        const double amplitude_plus  (10*std::log10(std::abs(demod_msk_->gf_plus().x())));

        if (filter_amp_pm_.get() < 0) {
          filter_amp_pm_.reset(0.5*(amplitude_plus+amplitude_minus));
          filter_amp_center_.reset(amplitude_center);
        } else {
          filter_amp_pm_.process(0.5*(amplitude_plus+amplitude_minus));
          filter_amp_center_.process(amplitude_center);
        }
        const double amplitude(filter_amp_pm_.get());
        const double sn_db(amplitude - filter_amp_center_.get());

        if (is_signal) {
          ++signal_present_;
          
          const double delta_phase_rad(demod_msk_->delta_phase_rad()
                                       + 2*M_PI*demod_msk_->period_sec() * (delta_f_Hz() - offset_Hz));
          phase_ += delta_phase_rad;
          while (phase_ >= M_PI) phase_ -= 2*M_PI;
          while (phase_ < -M_PI) phase_ += 2*M_PI;
          // std::cout << "XXX A+-0: " << amplitude << " " << sn_db << " " << sn_db+amplitude
          //           << " P: " << 0.5*(demod_msk_->pll_plus().theta() + demod_msk_->pll_minus().theta())
          //           << " DeltaF: " << demod_msk_->delta_phase_rad() /2/M_PI/demod_msk_->period_sec()
          //           << " " << delta_phase_rad /2/M_PI/demod_msk_->period_sec()
          //           << " " << phase_
          //           << " " << fc_Hz() - sp->center_frequency_Hz()
          //           << std::endl;
          const time_duration
            dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz()*downscale_factor_);
          result_phase::sptr r(result_phase::make(name_, sp->approx_ptime()+dt, fc_Hz(), fm_Hz_, amplitude, sn_db, phase_));
          sp->put_result(r);
        }
      }
      
      // collect bit data
      if (not is_first_call && demod_msk_->bit_available()) {
        if (not result_bits_) {
          const time_duration
            dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz()*downscale_factor_);
          result_bits_ = result_bits::make(name_+"_bits", sp->approx_ptime()+dt, fc_Hz(), fm_Hz());
        }
        result_bits_->push_back(demod_msk_->current_bit());
        rtcm_decoder_.decode(demod_msk_->current_bit());
        if (result_bits_->size() == size_t(0.5+fm_Hz())) {
          result_bits_->set_quality(double(signal_present_));
          //           std::cout << name_ << " " << "quality= " << result_bits_->quality() << " " << signal_present_ << std::endl;
          
          if (record_bits_ && result_bits_->quality() > 0.5)
            sp->put_result(result_bits_);
          result_bits_.reset();
          signal_present_ = 0;
        }
      }
    }    
  }
  
  // virtual void dump(processor::result_base::sptr) {
  //   // to be overwritten in a derived class
  // }

protected:
  bool detect_msk(service::sptr sp,
                  aligned_vector<complex_type>::const_iterator i0,
                  aligned_vector<complex_type>::const_iterator i1,
                  double &baud,
                  double &delta_f,
                  double &sn) {
    const size_t length(std::distance(i0, i1));
    fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<float>(std::distance(i0, i1)));
    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz()/downscale_factor_, 2.*delta_f_Hz());
    
    std::cout << "ps: " << name_ << " " << length << " " << s.index2freq(length/2-1) << " " << s.index2freq(length/2) 
              << " minus " << fc_Hz() << " " <<  filter_.get_shift()*sp->sample_rate_Hz() << " " << sp->center_frequency_Hz() << std::endl;
    
    const frequency_vector<double> ps_plus(-250., -40., s, std::abs<float>);
    if (filter_minus_.x().empty())
      filter_minus_.init(sp->approx_ptime(), ps_plus);
    else
      filter_minus_.update(sp->approx_ptime(), ps_plus);
    
    const frequency_vector<double> ps_minus(40., 250., s, std::abs<double>);
    if (filter_plus_.x().empty())
      filter_plus_.init(sp->approx_ptime(), ps_minus);
    else
      filter_plus_.update(sp->approx_ptime(), ps_minus);
    
    double f_minus(0), sn_minus(0);
    const bool found_minus = find_peak(filter_minus_.x(), f_minus, sn_minus);
    if (!found_minus) return false;

    double f_plus(0), sn_plus(0);
    const bool found_plus = find_peak(filter_plus_.x(), f_plus, sn_plus);
    if (!found_plus) return false;

    try {
      const int ip[2] = {
        s.freq2index(f_minus),
        s.freq2index(f_plus)
      };

      for (int i=-3; i<4; ++i) {
        std::cout << "detect_msk: " << i << " "
                  << s.index2freq(i+ip[0]) << " "
                  << s.index2freq(i+ip[1]) << " "      
                  << std::arg(fftw_.getBin(i+ip[0])) - std::arg(fftw_.getBin(i+ip[1])) << " "
                  << std::abs(fftw_.getBin(i+ip[0])) << " "
                  << std::abs(fftw_.getBin(i+ip[1])) << " "
                  << std::endl;
      }
      sn      = 0.5*(sn_minus + sn_plus);
      delta_f = 0.5* (f_plus + f_minus);
      baud    = f_plus - f_minus;
      std::cout << "ps: " << delta_f << " " << sn << " " << baud << std::endl;
    } catch (const std::exception& e) {
      LOG_ERROR(e.what());
      return false;
    }
    return true;
  }

  bool find_peak(const frequency_vector<double>& psf, double &f, double &sn) const {
    frequency_vector<double>::const_iterator i_max;
    double max(-1);
    double sum(0);
    for (frequency_vector<double>::const_iterator i(psf.begin()), end(psf.end()); i!=end; ++i) {
      std::cout << "find_peak ps: " << name_ << boost::format("%8.3f %.2e") % i->first % i->second << std::endl;
      sum += i->second;
      if (i->second > max) {
        max = i->second;
        i_max = i;
      }
    }
    double sum_w(0), sum_wx(0);
    for (frequency_vector<double>::const_iterator i(i_max-3); i!=(i_max+4) && i!=psf.end(); ++i) {
      std::cout << "ps: " << name_ << boost::format("%8.3f %.2e") % i->first % i->second << std::endl;
      sum_w += i->second;
      sum_wx += i->first * i->second;
    }
    f  = sum_wx/sum_w;
    sn = i_max->second/sum*psf.size();
    std::cout << " --- " << f << " (" << sn << ") " << std::endl;
    return true;
  }
private:
  fft_type fftw_;
  Filter::Cascaded<frequency_vector<double> > filter_plus_;
  Filter::Cascaded<frequency_vector<double> > filter_minus_;
  const std::string name_;
  double            fc_Hz_;     // center frequency
  const double      fm_Hz_;     // baud
  const bool        record_bits_;
  const double      dwl_Hz_;
  const double      period_Sec_;
  const double      min_SN_db_;
  const double      max_offset_ppb_rms_;
  int               downscale_factor_;
  double            delta_f_Hz_;
  demod::msk::sptr  demod_msk_;
  fir_filter        filter_;
  filter::iir_lowpass_1pole<double, double> filter_amp_pm_;
  filter::iir_lowpass_1pole<double, double> filter_amp_center_;
  double            phase_;
  size_t            signal_present_;
  result_bits::sptr result_bits_; // accumulates bits
  rtcm_decoder      rtcm_decoder_;
} ;
/// @}
/// @}
#endif // _DEMOD_MSK_PROCESSOR_HPP_cm130214_
