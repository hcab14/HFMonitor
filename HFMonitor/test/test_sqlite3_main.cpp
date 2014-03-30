// -*- C++ -*-
// $Id$

#include <stdlib.h>

#include <cmath>
#include <iostream>
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
      : _stmt(NULL) {
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
    bool step() {
      const int result(sqlite3_step(_stmt));
      switch (result) {
      case SQLITE_ROW:
	return true;
      case SQLITE_DONE:
	reset();
	return false;
      default:
	ASSERT_THROW_SQLITE3(result);
	return false; // to avoid compiler warning
      }
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
    statement& bind_double(int index, double val) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_double(_stmt, index, val));
      return *this;
    }
    statement& bind_int(int index, int val) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_int(_stmt, index, val));
      return *this;
    }
    statement& bind_int64(int index, sqlite3_int64 val) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_int64(_stmt, index, val));
      return *this;
    }
    statement& bind_null(int index) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_null(_stmt, index));
      return *this;
    }
    statement& bind_blob(int index, const void* data, int n, void(*destructor)(void*)=SQLITE_TRANSIENT) {
      ASSERT_THROW_SQLITE3(sqlite3_bind_blob(_stmt, index, data, n, destructor));
      return *this;
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
    sqlite3_stmt *_stmt;
  } ;

} // namespace db


int main()
{
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
      db::statement s(c, "INSERT INTO tbl1 VALUES(?1,?2);");
      for (int i=0; i<10; ++i) {
	std::string t(str(boost::format("%.15f") % (M_PI*i)));
	s.bind_text(1, t);
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

}
