// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _IIR_HPP_cm110629_
#define _IIR_HPP_cm110629_

#include <vector>

namespace { // anonymous

  template<typename COEFF,
	   typename STATE,
	   size_t N>
  struct iter_add {
  typedef typename std::vector<COEFF>::const_iterator const_iterator_coeff;
  typedef typename std::vector<STATE>::const_iterator const_iterator_state;
    static STATE exec(const_iterator_coeff a,
		      const_iterator_coeff b,
		      const_iterator_coeff in,
		      const_iterator_coeff out) {
      return -(*a)*(*out) + (*b)*(*in) + iter_add<COEFF,STATE,N-1>::exec(++a, ++b, --in, --out);
    }
  } ;
  template<>
  template<typename COEFF,
	   typename STATE>
  struct iter_add<COEFF,STATE,0> {
  typedef typename std::vector<COEFF>::const_iterator const_iterator_coeff;
  typedef typename std::vector<STATE>::const_iterator const_iterator_state;
    static STATE exec(const_iterator_coeff a,
		      const_iterator_coeff b,
		      const_iterator_state in,
		      const_iterator_state out) {
      return -(*a)*(*out) + (*b)*(*in);
    }
  } ;

  template<typename FLOAT,
	   size_t N>
  struct iter_move {
    typedef typename std::vector<FLOAT>::iterator iterator;
    static void exec(iterator i, iterator j) {
      *i = *j; 
      iter_move<FLOAT, N-1>::exec(++i, ++j);
    }
  } ;
  template<>
  template<typename FLOAT>
  struct iter_move<FLOAT, 0> {
    typedef typename std::vector<FLOAT>::iterator iterator;
    static void exec(iterator i, iterator j) {
      *i = *j; 
    }
  } ;
} // anonymous namespace

template<typename COEFF,
	 typename STATE,
	 size_t N>
class iir {
public:
  typedef std::vector<COEFF> vector_type_coeff;
  typedef std::vector<STATE> vector_type_state;

  iir()
    : a_(N, 0)
    , b_(N, 0)
    , in_(N, 0)
    , out_(N, 0) { a_[0] = COEFF(1); }

  iir(const vector_type_coeff& a,
      const vector_type_coeff& b)
    : a_(a)
    , b_(b)
    , in_ (a.size(), 0)
    , out_(a.size(), 0) {
    if (a_.size() != b_.size() && a_.size() != N) throw 1; // TODO
  }

  iir<COEFF,STATE,N>& init(const vector_type_coeff& a,
                           const vector_type_coeff& b) {
    if (a_.size() != a.size() && b_.size() != b.size()) throw 1; // TODO
    std::copy(a.begin(), a.end(), a_.begin());
    std::copy(b.begin(), b.end(), b_.begin());
    reset();
    return *this;
  }

  void reset(STATE s=STATE(0)) {
    for (size_t i=0; i<N; ++i)
      in_[i] = out_[i] = STATE(0);
    in_[N-1] = s;
  }

  STATE process(STATE s) {
    in_[N-1]  = s;
    out_[N-1] = b_[0]*in_[N-1] + iter_add<COEFF, STATE, N-2>::exec(a_.begin()+1,
                                                                   b_.begin()+1,
                                                                   in_.end() -2,
                                                                   out_.end()-2);
    if (a_[0] != COEFF(1)) out_[N-1] /= a_[0];
    // old <- new
    iter_move<STATE, N-2>::exec( in_.begin(),  in_.begin()+1);
    iter_move<STATE, N-2>::exec(out_.begin(), out_.begin()+1);
    return out_[N-1];
  }

  STATE get() const { return out_[N-1]; }
protected:
private:
  vector_type_coeff a_;
  vector_type_coeff b_;
  vector_type_state in_;
  vector_type_state out_;
} ;

#endif // _IIR_HPP_cm110629_
