// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2014 Christoph Mayer
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
#ifndef _SQLITE3_HPP_cm140413_
#define _SQLITE3_HPP_cm140413_

#include <iostream>
#include <iterator>
#include <string>
#include <stdexcept>

#include <boost/current_function.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <sqlite3.h>

#include "db/base.hpp"

#ifndef THROW_SITE_INFO
#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" + BOOST_STRINGIZE(__LINE__) + "\n")
#endif

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
  namespace sqlite3 {
 
   class context {
    public:
      context() { ASSERT_THROW_SQLITE3(sqlite3_initialize()); }
      ~context() {  ASSERT_THROW_DESTRUCTOR_SQLITE3(sqlite3_shutdown()); }
    protected:
    private:
    } ;

    class lock;
    class mutex {
    public:
      friend class lock;
      mutex()
        : _mutex(sqlite3_mutex_alloc(SQLITE_MUTEX_FAST)) {}
      ~mutex() { sqlite3_mutex_free(_mutex); }
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
      ~lock() { sqlite3_mutex_leave(_mutex.get()); }
    protected:
    private:
      mutex& _mutex;
    } ;

    class statement;
    class connection : public db::base {
    public:
      typedef boost::shared_ptr<connection> sptr;
      typedef ::sqlite3 impl_type;

      friend class statement;

      virtual ~connection() {
        ASSERT_THROW_DESTRUCTOR_SQLITE3(sqlite3_close(_db));
      }
      static sptr make(std::string filename,
                       int flags=(SQLITE_OPEN_URI       |
                                  SQLITE_OPEN_READWRITE |
                                  SQLITE_OPEN_CREATE)) {
        return sptr(new connection(filename, flags));
      }
      
    protected:
      impl_type* get() { return _db; }
      bool autocommit_mode() const { return (0 != sqlite3_get_autocommit(_db)); }

    private:
      connection(std::string filename, int flags)
        : _db(NULL) {
        ASSERT_THROW_SQLITE3(sqlite3_open_v2(filename.c_str(), &_db, flags, NULL));
        ASSERT_THROW_SQLITE3(sqlite3_extended_result_codes(_db, 1));
      }

      impl_type *_db;
    } ;

    class statement {
    public:
      statement(base::sptr db, std::string query)
        : _db(boost::dynamic_pointer_cast<connection>(db))
        , _stmt(NULL) {
        if (!_db.lock()) throw std::runtime_error("db::sqlite3::statement: non-sqlite3 db sptr passed");
        ASSERT_THROW_SQLITE3(sqlite3_prepare_v2(_db.lock()->get(),
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
        if (_db.lock()->autocommit_mode())
          return;
        try {
          statement s(_db.lock(), "ROLLBACK;");
          s.step(false);
        } catch (const std::exception &e) {
          // ignore
          std::cerr << "ROLLBACK failed: " << e.what() << std::endl;
        }
      }

      // column access    
      int column_count() const { return sqlite3_column_count(_stmt); }
      int           column_type  (int icol) const { return sqlite3_column_type  (_stmt, icol); }
      double        column_double(int icol) const { return sqlite3_column_double(_stmt, icol); }
      int           column_int   (int icol) const { return sqlite3_column_int   (_stmt, icol); }
      sqlite3_int64 column_int64 (int icol) const { return sqlite3_column_int64 (_stmt, icol); }
      int           column_bytes (int icol) const { return sqlite3_column_bytes (_stmt, icol); }
      const void*   column_blob  (int icol) const { return sqlite3_column_blob  (_stmt, icol); }
      std::string   column_text  (int icol) const {
        return std::string(reinterpret_cast<const char*>(  sqlite3_column_text  (_stmt, icol)));
      }
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
      boost::weak_ptr<connection> _db;
      sqlite3_stmt *_stmt;
    } ;

  } // namespace sqlite3
} // namespace db

// don't throw stones into the neighbour's garden
#undef ASSERT_THROW_SQLITE3
#undef ASSERT_THROW_DESTRUCTOR_SQLITE3

#endif //  _SQLITE3_HPP_cm140413_
