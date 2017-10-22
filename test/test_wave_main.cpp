// -*- C++ -*-
// $Id$

#include <string>
#include <iostream>
#include <sstream>

#include <memory>

#include <boost/property_tree/xml_parser.hpp>

#include "wave/reader.hpp"

#include "filter/fir_filter.hpp"
#include "filter/fir/polyphase_filter.hpp"
#include "filter/fir/fractional_resampler.hpp"

#include "filter/iir.hpp"
#include "filter/pll.hpp"

#include "run.hpp"

namespace demod {

  class fm {
  public:
    typedef std::complex<float> complex_type;

    fm()
      : x_()
      , x_last_()
      , work_()
      , fm_() {}

    const aligned_vector<float>& out() const { return fm_; }

    void process(aligned_vector<complex_type>::const_iterator i0,
                 aligned_vector<complex_type>::const_iterator i1) {

      const size_t num_samples(std::distance(i0, i1));
      resize(num_samples);

      x_last_[0] = x_.back();
      std::copy(i0, i1,   x_.begin());
      std::copy(i0, i1-1, x_last_.begin()+1);

      volk_32fc_x2_multiply_conjugate_32fc(&work_[0],
                                           &x_[0],
                                           &x_last_[0],
                                           num_samples);
      volk_32fc_s32f_atan2_32f(&fm_[0],
                               &work_[0],
                               1.0f,
                               num_samples);
    }

  protected:
    void resize(size_t n) {
      if (x_.size() != n) {
        x_.resize(n);
        x_last_.resize(n);
        work_.resize(n);
        fm_.resize(n);
      }
    }

  private:
    // FM demodulation
    aligned_vector<complex_type> x_;
    aligned_vector<complex_type> x_last_;
    aligned_vector<complex_type> work_;
    aligned_vector<float >       fm_;
  } ;

  // FM RDS extraction
  class fm_rds {
  public:
    typedef std::complex<float> complex_type;

    typedef filter::loop_filter_2nd<float> loop_filter;
    typedef filter::integrator_modulo<float> integrator;
    typedef filter::pll<float, loop_filter, integrator> pll_type;

    fm_rds(int num_samples,
           int sample_rate,
           int decim,
           int index)
      : decim_(decim)
      , fm_()
      , filter_(num_samples, 0.76/decim)
      , rds_   (num_samples/decim, 0)
      , frac_resampler_(19, 20, 5, num_samples/decim/20)
      , rds_resampled_(num_samples/decim*19/20, 0)
      , pll_zero_ (make_pll(sample_rate/decim*19/20,     0.0f, 4.0f, 1.0f/sqrt(2)))
      , rds_real_counter_(0)
      , taps_(make_pulse_filter(20,4,1.0f))
      , rds_real_(2*taps_.size())
      , rds_real_filtered_()
      , synch_(8) // hard-coded
      , synch_abs_(8) // hard-coded
      , index_(index) {
      filter_.shift(-57e3/sample_rate);
      assert((num_samples%decim) == 0);
      assert((num_samples/decim % 20) == 0);
      std::cout << "__TEST__ " << sample_rate/decim*19/20 << " " << 2375.0f << " "
                << 2375.0f/(sample_rate/decim*19/20) << std::endl;
    }

    void process(aligned_vector<complex_type>::const_iterator i0,
                 aligned_vector<complex_type>::const_iterator i1) {

      const size_t num_samples(std::distance(i0, i1));

      // (1) FM demodulation
      fm_.process(i0, i1);
      const aligned_vector<float>& fm_out = fm_.out();

      // (2) frequency xlating + decimating FIR filter
      for (int k=0,l=0; k<num_samples; ++k) {
        std::cout << "F" << index_ << " " << std::scientific << fm_out[k] << "\n";
        filter_.insert(std::complex<float>(fm_out[k], 0.0f));
        if ((k % decim_) == 0) {
          const auto y = filter_.process();
          std::cout << "G" << index_ << " " << y.real() << " " << y.imag() << "\n";
          rds_[l++] = y;
        }
      }

      // (3) fractional resampler
      frac_resampler_.process(rds_.begin(), rds_.end());

      // (4) carrier frequency correction
      rds_real_filtered_.resize(frac_resampler_.out().size());
      for (const complex_type y : frac_resampler_.out()) {
        std::cout << "U" << index_ << " " << y.real() << " " << y.imag() << "\n";
        const complex_type y2 = y*y;
        pll_zero_.process(y2);
        const float_t phase_carrier0 = 0.5*(pll_zero_.theta());
        const complex_type w = y*std::exp(complex_type(0, -phase_carrier0));
        std::cout << "P" << index_ << " "
                  << pll_zero_.theta() << " "
                  << pll_zero_.uf() << " "
                  << pll_zero_.f1() << " "
                  << w.real() << " " << w.imag() << "\n";
        rds_real_[rds_real_counter_] = rds_real_[rds_real_counter_+taps_.size()] = w.real();
        ++rds_real_counter_;
        rds_real_counter_ %= taps_.size();
        float result = 0.0f;
        volk_32f_x2_dot_prod_32f(&result,
                                 &rds_real_[rds_real_counter_],
                                 &taps_[0],
                                 taps_.size());
        const size_t block_length = 1000; // hard-coded
        size_t min_length = block_length;
        for (int j=0; j<8; ++j)
          min_length = std::min(min_length, synch_[j].size());
        if (min_length == block_length) {
          // obtain synchronization and push bits to next stage
          std::vector<float> mean(8), stddev(8);
          for (int j=0; j<8; ++j) {
            volk_32f_stddev_and_mean_32f_x2(&stddev[j], &mean[j],
                                            &synch_abs_[j][0],
                                            synch_abs_[j].size());
          }
          std::cout << "SYNC" << index_ << "    ";
          for (int j=0; j<8; ++j)
            std::cout << "(" << synch_abs_[j].size() << " " << mean[j] << ", " << stddev[j] << ") ";
          std::cout << "\n";

          // compute mean/stddev for abs(msg)
          std::transform(stddev.begin(), stddev.end(), mean.begin(), mean.begin(),
                         [](float x, float y) {
                           return y/x;
                         });
          std::cout << "SYNC" << index_ << " " << std::distance(mean.begin(), im) << " | ";

          const float threshold = 2.0; // hard-coded
          std::vector<float>::iterator im = std::max_element(mean.begin(), mean.end());
          if (*im > threshold)
            extract_bits(synch_[std::distance(mean.begin(), im)]);

          // clear buffers
          for (int j=0; j<8; ++j) {
            std::cout << mean[j] << " ";
            synch_[j].clear();
            synch_abs_[j].clear();
          }
          std::cout << "\n";
        }
        synch_    [rds_real_counter_ % 8].push_back(         result );
        synch_abs_[rds_real_counter_ % 8].push_back(std::abs(result));
      }

    }
  protected:
    void extract_bits(const aligned_vector<float>& s) {
      
    }
    static aligned_vector<float> make_pulse_filter(int   n,
                                                   int   m,
                                                   float beta) {
      filter::fir::raised_cosine<float> f(n);
      f.design(m, beta);

      aligned_vector<float> b(n+m, 0.0f);
      std::copy     (f.coeff().begin(), f.coeff().end(), b.begin());
      std::transform(f.coeff().begin(), f.coeff().end(), b.begin()+4, b.begin()+4,
                     [](float x, float y) -> float { return -x+y; });
      std::reverse(b.begin(), b.end());
      return b;
    }
    static pll_type make_pll(double fs_Hz,
                             double fc_Hz,
                             double dwl_Hz,
                             double xi) {
      loop_filter l(xi, dwl_Hz, fs_Hz);
      integrator  i(8*M_PI);
      return pll_type(fc_Hz, fs_Hz, dwl_Hz, l, i);
    }
  private:
    int decim_;

    // FM demodulation
    fm                           fm_;

    // RDS extraction
    fir_filter                   filter_;
    aligned_vector<complex_type> rds_;
    fractional_resampler         frac_resampler_;
    aligned_vector<complex_type> rds_resampled_;
    pll_type                     pll_zero_;

    // pulse shaping FIR filter
    int                          rds_real_counter_;
    aligned_vector<float>        rds_real_;
    aligned_vector<float>        taps_;
    aligned_vector<float>        rds_real_filtered_;

    // bit synchronization
    std::vector<aligned_vector<float> > synch_;
    std::vector<aligned_vector<float> > synch_abs_;
    int index_;
  } ;
};

class fm_rds_proc : public processor::base_iq {
public:
  fm_rds_proc(const boost::property_tree::ptree& config)
    : base_iq(config)
    , pf_()
    , demod_() {}

  virtual ~fm_rds_proc() {}

  void process_iq(processor::service_iq::sptr sp, const_iterator i0, const_iterator i1) {
    std::cout << "process_iq nS=" << std::distance(i0, i1)
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;
    const int decim        =   40;
    const int num_blocks   = std::distance(i0, i1)/decim;
    const int num_channels =  100;
    const int num_taps     = 4800;
    const int decim_rds    =   25;
    if (!pf_) {
      // num_blocks*decim = 1000000
      pf_ = std::make_unique<polyphase_filter>(num_blocks, decim, num_channels, num_taps,
                                               150e3/sp->sample_rate_Hz()*decim);
      demod_.resize(num_channels);
      x_.resize(num_blocks);
      for (int i=0; i<num_channels; ++i)
        demod_[i] = std::make_unique<demod::fm_rds>(num_blocks, sp->sample_rate_Hz()/decim, decim_rds, i);
    }
    std::cout << "TTEST: " << std::distance(i0, i1) << " " << pf_->num_blocks()*pf_->decim() << " "
              << decim << " " << num_blocks << " " << num_channels << " " << num_taps << " TT: " << (std::distance(i0, i1) % decim) << " "
              << (std::distance(i0, i1)/decim % decim_rds) << " "
              << (num_taps % decim) << std::endl;

    pf_->process(i0, i1);

    const aligned_vector<std::complex<float> >& out = pf_->out();

    for (int j=0; j<pf_->num_channels(); ++j) {
      if (j!=17 && j!=10 && j!=23 && j!=99 && j!=29 && j!=45 && j!=33 && j!=29)
        continue;

      for (int k=0; k<pf_->num_blocks(); ++k) {
        x_[k] = out[k*pf_->num_channels()+j];
        std::cout << "A" << j << " " << x_[k].real() << " " << x_[k].imag() << "\n";
      }
      demod_[j]->process(x_.begin(), x_.end());
    } // next channel
  }

private:
  std::unique_ptr<polyphase_filter> pf_;
  aligned_vector<complex_type> x_;
  std::vector<std::unique_ptr<demod::fm_rds> > demod_;
} ;


int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_wave");
  try {
    const std::string filename("test/wave.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    wave::reader_iq<fm_rds_proc> r(config.get_child("Wave"));

    for (int i=1; i<argc; ++i)
      r.process_file(argv[i]);

    r.finish();

  } catch (const std::exception &e) {
    LOG_ERROR(e.what());
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
