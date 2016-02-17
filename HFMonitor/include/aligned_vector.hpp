// -*- C++ -*-
// $Id$

#ifndef _ALIGNED_VECTOR_cm150813_
#define _ALIGNED_VECTOR_cm150813_

#include <volk/volk.h>
#include <volk/volk_prefs.h>

template<typename T>
class aligned_vector {
public:
  aligned_vector(size_t n=0)
    : _n(0)
    , _p(NULL) {
    resize(n);
  }
  aligned_vector(size_t n, T init)
    : _n(0)
    , _p(NULL) {
    resize(n, init);
  }
  ~aligned_vector() {
    volk_free(_p);
  }

  typedef T* iterator;
  typedef const T* const_iterator;
  
  size_t size() const { return _n; }

  operator T*() { return _p; }
  operator const T*() const { return _p; }

  T& operator[](size_t i) { return _p[i]; }
  const T& operator[](size_t i) const { return _p[i]; }

  T* begin() { return _p; }
  T* end() { return _p+_n; }
  const T* begin() const { return _p; }
  const T* end() const { return _p+_n; }
  
  void resize(size_t n) {
    if (_n == n) return;
    volk_free(_p);
    _n = n;
    _p = reinterpret_cast<T*>(volk_malloc(n*sizeof(T), volk_get_alignment()));
  }
  void resize(size_t n, T t0) {
    resize(n);
    for (size_t i(0), n(size()); i<n; ++i)
      _p[i] = t0;
  }
protected:
private:
  size_t _n; // length
  T     *_p; // pointer to data
} ;

#endif // _ALIGNED_VECTOR_cm150813_
