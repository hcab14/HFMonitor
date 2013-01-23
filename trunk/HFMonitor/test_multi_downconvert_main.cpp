// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "network/broadcaster.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "multi_downconvert_processor.hpp"
#include "processor/registry.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

#include "network/protocol.hpp"
#include "wave/writer.hpp"

template<typename FFTFloat>
class multi_downconvert_toBC : public multi_downconvert_processor<FFTFloat> {
public:
  typedef boost::shared_ptr<multi_downconvert_toBC<FFTFloat> > sptr;
  typedef typename multi_downconvert_processor<FFTFloat>::ptree ptree;
  typedef typename multi_downconvert_processor<FFTFloat>::service service;
  typedef typename multi_downconvert_processor<FFTFloat>::filter_param filter_param;
  typedef typename multi_downconvert_processor<FFTFloat>::overlap_save_type overlap_save_type;
  multi_downconvert_toBC(const ptree& config)
    : multi_downconvert_processor<FFTFloat>(config) {}
  virtual ~multi_downconvert_toBC() {}

protected:

private:
  virtual void dump(typename service::sptr sp,
                    const filter_param& fp,
                    const typename overlap_save_type::complex_vector_type& out) {
    std::cout << "multi_downconvert_toBC::dump: " << fp.name() << " " << out.size() << std::endl;

    typedef typename overlap_save_type::complex_vector_type complex_vector_type;
    const size_t bytes_per_sample(3);
    // data
    std::ostringstream oss;
    for (typename complex_vector_type::const_iterator i(out.begin()); i!=out.end(); ++i) {
      wave::detail::write_real_sample(oss, bytes_per_sample, i->real());
      wave::detail::write_real_sample(oss, bytes_per_sample, i->imag());
    }
    iq_info h_iq(sp->sample_rate_Hz()/fp.decim(), 
                 sp->center_frequency_Hz()+fp.center_freq_Hz(),
                 'I', bytes_per_sample, 0, 0);
    std::string data;
    std::copy(h_iq.begin(), h_iq.end(), std::back_inserter(data));
    data += oss.str();

    // header
    boost::uint32_t stream_number(0); //TODO
    header h("WAV_0000", sp->approx_ptime(), stream_number, data.size());
    std::string bytes;
    std::copy(h.begin(), h.end(), std::back_inserter(bytes));
    bytes += data;
    // bc->bc_data(sp->approx_ptime(), fp.name(), bytes);    
  }
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_config_multi_downconvert");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_multi_downconverter.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<multi_downconvert_toBC<double> > > >
      c(config.get_child("MultiDownConverter"));
    
    
    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    c.start();
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
