// -*- C++ -*-
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
#ifndef BACKGROUND_ESTIMATOR_HPP_cm140413_
#define BACKGROUND_ESTIMATOR_HPP_cm140413_

#include "Spectrum.hpp"
#include "carrier_monitoring/polynomial_interval_fit.hpp"
#include "vector_compressor.hpp"

class background_estimator {
public:
  typedef frequency_vector<double> fvector;
  typedef fvector::iterator fvector_iterator;
  typedef fvector::const_iterator fvector_const_iterator;
  typedef std::vector<double> index_vector_type;

  background_estimator(size_t length,
		       size_t poly_degree,
		       size_t num_intervals)
    : _pif(poly_degree, make_indices(num_intervals, length))
    , _xs(length, 0)
    , _spec_db(length, 0)
    , _spec_db_fitted(length, 0)
    , _b(length, 1) {
    for (size_t i(0), n(_xs.size()); i<n; ++i)
      _xs[i] = i;
  }
  ~background_estimator() {}

  const std::vector<double>& spec_db() const { return _spec_db; }
  double spec_db(size_t index) const { return _spec_db[index]; }

  const std::vector<double>& spec_db_fitted() const { return _spec_db_fitted; }
  double spec_db_fitted(size_t index) const { return _spec_db_fitted[index]; }

  const polynomial_interval_fit& get_pif() const { return _pif; }

  void update(size_t length) {
    const size_t poly_degree(_pif.poly_degree());
    const index_vector_type& iv(_pif.t_indices());
    std::vector<double> intervals(iv.size(), 0); // interval definitions [0..1]
    for (size_t i(0), n(iv.size()); i<n; ++i)
      intervals[i] = double(iv[i])/double(_spec_db.size());

    _pif = polynomial_interval_fit(poly_degree, make_indices(intervals, length));
    _xs.resize(length, 0);
    for (size_t i(0), n(_xs.size()); i<n; ++i)
      _xs[i] = i;
    _spec_db.resize(length, 0);
    _spec_db_fitted.resize(length, 0);
    _b.resize(length, 1);
  }

  bool do_fit(const fvector& xf,
	      double threshold = 2.5) {
    // resize if needed
    if (_spec_db.size() != xf.size())
      update(xf.size());

    // initialization
    for (size_t i(0), n(_spec_db.size()); i<n; ++i) {      
      _spec_db[i] = 10*log10(xf[i].second);    
      _spec_db_fitted[i] = 0;
      _b[i] = 1;
    }

    // iterations
    const size_t max_iter(100);
    size_t nchanged(1);
    for (size_t l(0); l<max_iter && nchanged; ++l) {
      if (!_pif.fit(_xs, _spec_db, _b)) {
	std::cerr << "fit failed" << std::endl;
	break;
      }
      nchanged = 0;
      for (size_t i(0), n(_spec_db.size()); i<n; ++i) {
	_spec_db_fitted[i] = _pif.eval(i).first;
	const bool over_threshold(_spec_db[i]-_spec_db_fitted[i] > threshold);
	nchanged += !(over_threshold != _b[i]); // TO BE CHECKED
	_b[i] = !over_threshold;
      }
    }
    return (nchanged==0);
  }

  // makes a background subtracted compressed spectrum quantized in 1db steps
  // the threshold is increased until the occupancy is reached.
  std::pair<vector_compressor::vector_type, char> make_spectrum(char   min_threshold,
								double max_occupancy) {
    const size_t n(_spec_db.size());
    std::vector<char> bytes(n, 0);
    vector_compressor::vector_type compressed_bytes(n, 0);
    const size_t imo(size_t(0.5+max_occupancy*n));
    char threshold(min_threshold);
    for (; threshold<127 && compressed_bytes.size() >= imo; ++threshold) {
      for (size_t i(0); i<n; ++i) {
	// clamp subtracted signal in [0,100]
	const char s(char(std::min(std::max(0., _spec_db[i]-_spec_db_fitted[i]), 100.)));
	if (i==10)
	  std::cout << "SPEC: " << _spec_db[i] << " " << _spec_db_fitted[i] << " " << int(s) << " " << int(threshold) << std::endl;
	bytes[i]     = (s>=threshold) ? s : 0;
      }
      compressed_bytes = _compressor.compress(bytes);
      std::cout << "OCC,THR " << double(compressed_bytes.size())/n << " " << int(threshold) << std::endl;
    }
    return std::make_pair(compressed_bytes, threshold-1);
  }

protected:
  static index_vector_type make_indices(size_t m,   // number of intervals
					size_t n) { // length of spectrum vector
    index_vector_type indices(m+1, 0);
    for (size_t i(0); i<m; ++i)
      indices[i] = (i*n)/m;
    indices[m] = n-1;
    indices[0] = std::min(indices[1],   indices[0]+size_t(0.005*n));
    indices[m] = std::max(indices[m-1], indices[m]-size_t(0.005*n));    
    return indices;
  }
  static index_vector_type make_indices(const std::vector<double>& intervals, // intervals [0..1]
					size_t n) {                           // length of spectrum vector
    const size_t m(intervals.size());
    index_vector_type indices(m, 0);
    for (size_t i(0); i<m; ++i)
      indices[i] = size_t(intervals[i]*n);
    return indices;
  }

private:
  polynomial_interval_fit _pif;
  std::vector<double>     _xs;              // 
  std::vector<double>     _spec_db;         // spectrum in db
  std::vector<double>     _spec_db_fitted;  // fitted spectrum
  std::vector<size_t>     _b;               // b[i] == 1 (0) --> point is (not) used in fit
  vector_compressor       _compressor;      //
} ;

#endif // BACKGROUND_ESTIMATOR_HPP_cm140413_

