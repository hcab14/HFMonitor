// -*- C++ -*-
// $Id$

#include <stdlib.h>

#include <cmath>
#include <iostream>
#include <iterator>
#include <string>
#include <stdexcept>

#include <bzlib.h>

#include <boost/current_function.hpp>
#include <boost/format.hpp>

#include <sqlite3.h>

#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" + BOOST_STRINGIZE(__LINE__) + "\n")

#define ASSERT_THROW_SQLITE3(_x)                                        \
  if ((_x))								\
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x) \
                                             + " msg: " + std::string(sqlite3_errstr(_x)))); \
  else void(0) 

#define ASSERT_THROW_DESTRUCTOR_SQLITE3(_x)				\
  if ((_x) && !std::uncaught_exception())				\
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x) \
                                             + " msg: " + std::string(sqlite3_errstr(_x)))); \
  else void(0) 


namespace db {
  class sqlite3_context {
  public:
    sqlite3_context() { ASSERT_THROW_SQLITE3(sqlite3_initialize()); }
    ~sqlite3_context() {  sqlite3_shutdown(); }

  protected:
  private:
  } ;

  class lock;
  class mutex {
  public:
    friend class lock;
    mutex()
      : _mutex(sqlite3_mutex_alloc(SQLITE_MUTEX_FAST)) {}
    ~mutex() {
      sqlite3_mutex_free(_mutex);
    }
  protected:
    sqlite3_mutex* get() { return _mutex; }
  private:
    sqlite3_mutex *_mutex;
  } ;

  class lock {
  public:
    lock(mutex& m)
    : _mutex(m) {
      sqlite3_mutex_enter(_mutex.get());
    }
    ~lock() {
      sqlite3_mutex_leave(_mutex.get());
    }
  protected:
  private:
    mutex& _mutex;
  } ;

  class statement;
  class connection {
  public:
    friend class statement;
    connection(std::string filename, int flags=(SQLITE_OPEN_URI       |
						SQLITE_OPEN_READWRITE |
						SQLITE_OPEN_CREATE))
      : _db(NULL) {
      ASSERT_THROW_SQLITE3(sqlite3_open_v2(filename.c_str(), &_db, flags, NULL));
      ASSERT_THROW_SQLITE3(sqlite3_extended_result_codes(_db, 1));
    }
    ~connection() {
      ASSERT_THROW_DESTRUCTOR_SQLITE3(sqlite3_close(_db));
    }
  protected:
    sqlite3* get() { return _db; }

  private:
    sqlite3 *_db;
  } ;

  class statement {
  public:
    statement(connection& db, std::string query)
      : _db(db)
      , _stmt(NULL) {
      ASSERT_THROW_SQLITE3(sqlite3_prepare_v2(db.get(),
					      query.c_str(),
					      -1,//query.size(),
					      &_stmt,
					      NULL));      
    }
    ~statement() {
      ASSERT_THROW_DESTRUCTOR_SQLITE3(sqlite3_finalize(_stmt));
    }
    void reset() {
      ASSERT_THROW_SQLITE3(sqlite3_reset(_stmt));
    }
    // returns true if data is available
    // throws on error
    bool step(bool do_rollback_if_error=true) {
      const int result(sqlite3_step(_stmt));
      switch (result) {
      case SQLITE_ROW:
	return true;
      case SQLITE_DONE:
	reset();
	return false;
      case SQLITE_FULL:      // database or disk full
      case SQLITE_IOERR:     // disk I/O error
      case SQLITE_BUSY:      // database in use by another process
      case SQLITE_NOMEM:     // out or memory
      case SQLITE_INTERRUPT: // processing interrupted by application request
	if (do_rollback_if_error) {
	  reset();
	  do_rollback();
	}
	return false;
      default:
	ASSERT_THROW_SQLITE3(result);
	return false; // to avoid compiler warning
      }
    }

    void do_rollback() {
      statement s(_db, "ROLLBACK;");
      s.step(false);
    }

    // column access    
    int column_count() const { return sqlite3_column_count(_stmt); }
    int           column_type  (int icol) const { return sqlite3_column_type  (_stmt, icol); }
    double        column_double(int icol) const { return sqlite3_column_double(_stmt, icol); }
    int           column_int   (int icol) const { return sqlite3_column_int   (_stmt, icol); }
    sqlite3_int64 column_int64 (int icol) const { return sqlite3_column_int64 (_stmt, icol); }
    int           column_bytes (int icol) const { return sqlite3_column_bytes(_stmt, icol); }
    const void*   column_blob  (int icol) const { return sqlite3_column_blob (_stmt, icol); }
    std::string   column_text  (int icol) const { return std::string(reinterpret_cast<const char*>(sqlite3_column_text (_stmt, icol))); }

    // sqlite3_column_value()
    // sqlite3_column_text16()
    // sqlite3_column_bytes16()

    // binding
    statement& bind_double(std::string name, double val) {
      return bind_double(sqlite3_bind_parameter_index(_stmt, name.c_str()), val);
    }
    statement& bind_double(int index, double val) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_double(_stmt, index, val));
      return *this;
    }
    statement& bind_int(std::string name, int val) {
      return bind_int(sqlite3_bind_parameter_index(_stmt, name.c_str()), val);
    }
    statement& bind_int(int index, int val) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_int(_stmt, index, val));
      return *this;
    }
    statement& bind_int64(std::string name, sqlite3_int64 val) {
      return bind_int64(sqlite3_bind_parameter_index(_stmt, name.c_str()), val);
    }
    statement& bind_int64(int index, sqlite3_int64 val) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_int64(_stmt, index, val));
      return *this;
    }
    statement& bind_null(std::string name) {
      return bind_null(sqlite3_bind_parameter_index(_stmt, name.c_str()));
    }
    statement& bind_null(int index) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_null(_stmt, index));
      return *this;
    }
    statement& bind_blob(std::string name, const void* data, int n) {
      return bind_blob(sqlite3_bind_parameter_index(_stmt, name.c_str()), data, n);
    }
    statement& bind_blob(int index, const void* data, int n, void(*destructor)(void*)=SQLITE_TRANSIENT) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_blob(_stmt, index, data, n, destructor));
      return *this;
    }
    statement& bind_text(std::string name, std::string s, void(*destructor)(void*)=SQLITE_TRANSIENT) {
      return bind_text(sqlite3_bind_parameter_index(_stmt, name.c_str()), s, destructor);
    }
    statement& bind_text(int index, std::string s, void(*destructor)(void*)=SQLITE_TRANSIENT) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_text(_stmt, index, s.c_str(), s.size(), destructor));
      return *this;
    }
    // int sqlite3_bind_text16(sqlite3_stmt*, int, const void*, int, void(*)(void*));
    // int sqlite3_bind_value(sqlite3_stmt*, int, const sqlite3_value*);
    // int sqlite3_bind_zeroblob(sqlite3_stmt*, int, int n);

  protected:
  private:
    connection&   _db;
    sqlite3_stmt *_stmt;
  } ;

} // namespace db

class compressed_vector {
public:
  typedef std::vector<char> vector_type;
  typedef vector_type::iterator iterator;
  typedef vector_type::const_iterator const_iterator;

  compressed_vector(const vector_type& v) {
    _v.resize(2*v.size());
    iterator j(_v.begin()), k(j);
    *j = 0;
    for (const_iterator i(v.begin()), end(v.end()); i!=end; ++i) {
      if (*i == 0) {
	if (*j == -127 || *j > 0) {
	  j = (*j > 0) ? k : j+1;
	  *j = 0;
	}
	*j -= 1;
      } else { // *i != 0
	if (*j < 0 || *j == 127) {	 
	  j = k+1;
	  *j = 0;
	  k = j+1;
	}
	*k = *i;
	++k;
	*j += 1;
      }
    }
    _v.resize(std::distance(_v.begin(), j+1));
    std::cout << "compression factor = " << double(_v.size())/double(v.size()) << std::endl;
    size_t n=_v.size();
    unsigned int dest_len(32*n);
    std::vector<char> vc(32*n,0);
    int result = BZ2_bzBuffToBuffCompress(&vc[0], &dest_len, &_v[0], n, 9, 0, 0);
    std::cout << "bzip2: " << result << " " << dest_len << std::endl;

  }
  ~compressed_vector() {}

  const vector_type& get() const { return _v; }
  vector_type decompress() const {
    vector_type v;
    for (const_iterator i(_v.begin()), end(_v.end()); i!=end;) {
      if (*i < 0) {
	for (size_t j(0), m(-*i); j<m; ++j)
	  v.push_back(0);
	++i;
      } else {
	const size_t m(*i);
	std::cout << "m= " << m << std::endl;
	++i;
	for (size_t j(0); j<m && i!=end; ++j) {
	  v.push_back(*i);
	  ++i;
	}
      }
    }
    return v;
  }
protected:
private:
  vector_type _v;
} ;

int main()
{
  size_t n(10000);
  std::vector<char> v(n,0);
  for (size_t i(0); i<100; ++i) {
    // v[100+1*i] =1+(lrand48()%128);
    // v[550+1*i] =1+(lrand48()%128);
    // v[777+i] = 1+(lrand48()%128);
  }

  compressed_vector cv(v);
  std::copy(cv.get().begin(), cv.get().end(), std::ostream_iterator<int>(std::cout, " "));
  std::cout << std::endl;

  unsigned int dest_len(2*n);
  std::vector<char> vc(2*n,0);
  int result = BZ2_bzBuffToBuffCompress(&vc[0], &dest_len, &v[0], n, 9, 0, 0);
  std::cout << "bzip2: " << result << " " << dest_len << std::endl;
  std::cout << "BZ2 compression factor = " << double(dest_len)/v.size() << std::endl;

  // std::vector<char> dcv(cv.decompress());
  // std::copy(dcv.begin(), dcv.end(), std::ostream_iterator<int>(std::cout, " "));
  // std::cout << std::endl;

  return 0;
#if 0
  try {
    db::sqlite3_context sqc;
    std::cout << sqlite3_libversion() << std::endl;
    std::cout << sqlite3_sourceid()   << std::endl;
    db::mutex m;
    db::lock l(m);
    db::connection c("file:data.db?mode=rwc");
    { db::statement s(c, "CREATE TABLE IF NOT EXISTS tbl1(one VARCHAR(10), two SMALLINT);");
      while (s.step()) std::cout << "step\n";
    }
    {
      db::statement s(c, "INSERT INTO tbl1 VALUES(@A,$2);");
      for (int i=0; i<1000; ++i) {
	std::string t(str(boost::format("%.15f") % (M_PI*i)));
	s.bind_text("@A", t);
	s.bind_int(2, i);

	while (s.step())
	  std::cout << "step\n";

      }
    }
    { db::statement s(c, "SELECT * FROM tbl1;");
      while (s.step()) {
	for (int i(0), n(s.column_count()); i<n; ++i)
	  std::cout << "( " << s.column_text(i) << " ) ";
	std::cout << std::endl;
      }
    }

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  const size_t n(1000);
  std::vector<char> s(n,0);
  std::vector<char> sc(10*n,0);
  s[2] = 1;
  s[3] = 2;
  s[4] = 2;
  s[5] = 2;
  s[6] = 2;
  s[200] = 2;
  s[654] = 4;
  for (size_t i(0); i<n; ++i)
    s[i] = (!(i%2))*(1+0*lrand48());

  unsigned int dest_len(2*n);
  int result = BZ2_bzBuffToBuffCompress(&sc[0], &dest_len, &s[0], n, 9, 0, 0);
  std::cout << "bzip2: " << result << " " << dest_len << std::endl;
  std::cout << "sizeof(s) = " << sizeof(s) << std::endl;
#endif
}
