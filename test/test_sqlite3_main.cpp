// -*- C++ -*-
// $Id$

#include <iostream>

#include "db/sqlite3.hpp"

int main()
{
  try {
    db::sqlite3::context sqc;
    std::cout << sqlite3_libversion() << std::endl;
    std::cout << sqlite3_sourceid()   << std::endl;
    db::sqlite3::mutex m;
    db::sqlite3::lock l(m);
    db::sqlite3::connection::sptr c_(db::sqlite3::connection::make("file:data.db?mode=rwc"));
    db::base::sptr c(c_);
    
    { db::sqlite3::statement s(c, "CREATE TABLE IF NOT EXISTS tbl1(one VARCHAR(10), two SMALLINT);");
      while (s.step()) std::cout << "step\n";
    }
    {
      db::sqlite3::statement s(c, "INSERT INTO tbl1 VALUES(@A,$2);");
      for (int i=0; i<1000; ++i) {
	std::string t(str(boost::format("%.15f") % (M_PI*i)));
	s.bind_text("@A", t);
	s.bind_int(2, i);

	while (s.step())
	  std::cout << "step\n";

	s.do_rollback();
      }
    }
    { db::sqlite3::statement s(c, "SELECT * FROM tbl1;");
      while (s.step()) {
	for (int i(0), n(s.column_count()); i<n; ++i)
	  std::cout << "( " << s.column_text(i) << " ) ";
	std::cout << std::endl;
      }
    }

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
