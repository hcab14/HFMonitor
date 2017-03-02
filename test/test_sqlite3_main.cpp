// -*- C++ -*-
// $Id$

#include <stdlib.h>

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "db/sqlite3.hpp"

class db_interface {
public:
  db_interface(std::string filename)
    : _connection
      (setup_tables
       (db::sqlite3::connection::make
	(str(boost::format("file:%s?mode=rwc") % filename))))
    , _select_timestamp(_connection, "SELECT id FROM epochs where timestamp==$timestamp;")
    , _insert_timestamp(_connection, "INSERT INTO epochs(timestamp) VALUES($timestamp);")
    , _select_channel_type(_connection, "SELECT id FROM channel_types where type==$type;")
    , _insert_channel_type(_connection, "INSERT INTO channel_types(type) VALUES($type);")
    , _select_channel(_connection, "SELECT id FROM channels where freq_Hz==$freq_Hz;")
    , _insert_channel(_connection, "INSERT INTO channels(freq_Hz, chan_type_id) VALUES($freq_Hz, $chan_type_id);")
    , _insert_peak
      (_connection,
       "INSERT INTO peaks(freq_Hz, width_Hz, strength_dB, SN_dB, epoch_id, chan_id) \n\
        VALUES($freq_Hz,$width_Hz,$strength_dB,$SN_dB,$epoch_id,$chan_id);")
  {
  }

  void insert_peak(std::string timestamp,
		   std::string type,
		   double freq_Hz) {
    db::sqlite3::lock lock(_mutex);

    const boost::int64_t epoch_id
      (insert_id
       (timestamp, _select_timestamp, _insert_timestamp, "$timestamp"));
    std::cout << "epoch_id = " << epoch_id << std::endl;
    const boost::int64_t chan_type_id
      (insert_id
       (type, _select_channel_type, _insert_channel_type, "$type"));
    std::cout << "chan_type_id = " << chan_type_id << std::endl;

    const boost::int64_t chan_id
      (insert_id
       (freq_Hz, _select_channel,
	_insert_channel.bind("$chan_type_id", chan_type_id), "$freq_Hz"));
    std::cout << "chan_id = " << chan_id << std::endl;

    _insert_peak
      .bind("$freq_Hz",     1510027.1)
      .bind("$width_Hz",    1.2)
      .bind("$strength_dB", -65.2)
      .bind("$SN_dB",       10.)
      .bind("$epoch_id",    epoch_id)
      .bind("$chan_id",     chan_id)
      .step_all();
  }

protected:
  // inserts a value and if it does not exist and returns the id of the value
  template<typename T>
  boost::int64_t insert_id(T value,
			  db::sqlite3::statement& select_statement,
			  db::sqlite3::statement& insert_statement,
			  std::string what) {
    select_statement.bind(what, value);
    boost::int64_t id(-1);
    if (select_statement.step()) { // value is already used
      id = select_statement.get_column<boost::int64_t>(0);
      select_statement.reset();
    } else { // new value
      insert_statement
	.bind(what, value)
	.step_all();
      id = _connection->last_insert_rowid();
    }
    return id;
  }

private:
  static db::sqlite3::connection::sptr setup_tables(db::sqlite3::connection::sptr c) {
    std::vector<std::string> cts;
    cts.push_back // channel_types
      ("CREATE TABLE IF NOT EXISTS            \n"
       "channel_types(id INTEGER PRIMARY KEY, \n"
       "              type VARCHAR(2) UNIQUE, \n"
       "              description VARCHAR(20));");
    cts.push_back // channels
      ("CREATE TABLE IF NOT EXISTS       \n"
       "channels(id INTEGER PRIMARY KEY  \n"
       "         freq_Hz REAL UNIQUE,    \n"
       "         chan_type_id INTEG      \n"
       "         FOREIGN KEY(chan_type_id) REFERENCES channel_types(id));");
    cts.push_back // epochs
      ("CREATE TABLE IF NOT EXISTS     \n"
       "epochs(id INTEGER PRIMARY KEY  \n"
       "       timestamp UNIQUE NOT NULL DEFAULT CURRENT_TIMESTAMP);");
    cts.push_back // peaks
      ("CREATE TABLE IF NOT EXISTS     \n"
       "peaks(id INTEGER PRIMARY KEY,  \n"
       "      freq_Hz REAL,            \n"
       "      width_Hz REAL,           \n"
       "      strength_dB REAL,        \n"
       "      SN_dB REAL,              \n"
       "      epoch_id INTEGER,        \n"
       "      chan_id INTEGER,         \n"
       "      FOREIGN KEY(epoch_id) REFERENCES epochs(id), \n"
       "      FOREIGN KEY(chan_id)  REFERENCES channels(id));");
    cts.push_back // spectra
      ("CREATE TABLE IF NOT EXISTS      \n"
       "spectra(id INTEGER PRIMARY KEY, \n"
       "        fMin_Hz REAL,           \n"
       "        df_Hz REAL,             \n"
       "        fMax_Hz REAL,           \n"
       "        n INTEGER,              \n"
       "        data BLOB,              \n"
       "        chan_id INTEGER,        \n"
       "        epoch_id INTEGER,       \n"
       "        FOREIGN KEY(epoch_id) REFERENCES epochs(id), \n"
       "        FOREIGN KEY(chan_id)  REFERENCES channels(id));");
    
    BOOST_FOREACH(const std::string& st, cts) {
      db::sqlite3::statement(c, st).step_all();
    }
    std::cout << "OK" << std::endl;
    return c;
  }

  const db::sqlite3::context _context;
  db::sqlite3::mutex _mutex;

  db::sqlite3::connection::sptr _connection;

  db::sqlite3::statement _select_timestamp;
  db::sqlite3::statement _insert_timestamp;

  db::sqlite3::statement _select_channel_type;
  db::sqlite3::statement _insert_channel_type;

  db::sqlite3::statement _select_channel;
  db::sqlite3::statement _insert_channel;
  db::sqlite3::statement _insert_peak;
} ;

int main()
{
  try {
    db_interface dbi("data.db");

    std::cout << "sqlite3_threadsafe=" << sqlite3_threadsafe() << std::endl;

    // insert random peaks
    const char* types[2] = { "EU", "NA" };

    for (size_t i(0); i<1000; ++i) {
      const int ts(lrand48() % 60);
      std::string timestamp(str(boost::format("2014-04-28 16:17:%d.123") % ts));
      const std::string type(types[lrand48() % 2]);
      dbi.insert_peak(timestamp, type, 1510.0);
    }

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
