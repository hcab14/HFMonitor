#include <iostream>

// Format:
// Header
//  SampleNumber (of 1st sample) (64 bit)
//  SampleRate / Samples/sec (32 bit)
//  DDCCenterFrequency / Hz (32 bit)
//  Number of Data Samples (32 bit)
// Data:
//  Sample I
//  Sample Q
// ...

#include <boost/cstdint.hpp>

class DataPacket {
public:
private:
  struct Header {
    Header(boost::unit64_t _sampleNumber,
	   boost::uint32_t _sampleRate;
	   boost::uint32_t _ddsCenterFrequency;
	   boost::uint32_t _numberOfSamples;  )
      : sampleNumber      (_sampleNumber)
      , sampleRate        (_sampleRate)
      , ddsCenterFrequency(_ddsCenterFrequency)
      , numberOfSamples   (_numberOfSamples) {}

    const uint8_t* asBuffer() const { return *this; }

    boost::unit64_t sampleNumber()       const { return sampleNumber_; }
    boost::uint32_t sampleRate()         const { return sampleRate_; }
    boost::uint32_t ddsCenterFrequency() const { return ddsCenterFrequency_; }
    boost::uint32_t numberOfSamples()    const { return numberOfSamples_; }
  } ;
} ;

int main()
{
  boost::uint64_t u(0);
  std::cout << u << " " << ~u << std::endl;

  return 1;
}
