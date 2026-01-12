#include <iostream>
#include <map>
#include <string>

#include "common/filesystem.h"
#include "common/sqlite.h"

using namespace worldview;

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "usage :" << argv[0] << " [dicovery_db]" << std::endl;
    return 0;
  }

  std::string pub_stmt_str = R"(
  select 
    PublicationData_topic_name, PublicationData_type_name, PublicationData_type
  from 
    DCPSPublication;
  )";

  auto conn = std::make_shared<Sqlite::Conn>(argv[1]);
  Sqlite::Query q(conn, pub_stmt_str);
  std::cout << "num cols:" << q.column_count();
  for (int i = 0; i < q.column_count(); ++i) {
    std::cout << " " << q.column_name(i) << "(" << q.column_decltype(i) << ")";
  }
  std::cout << std::endl;
  for (auto it = q.begin(); it != q.end(); ++it) {
    std::cout << "cols " << it->data_count() << ": ";
    for (int i = 0; i < it->data_count(); i++) {
      std::cout << "type(" << i << ")=" << it->column_type_str(i) << " ";
    }
    std::cout << it->get<std::string>(0) << " " << (*it).get<std::string>(1)
              << std::endl;
  }

  return 0;
}