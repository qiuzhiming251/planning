#pragma once
#include <sqlite3.h>
#include <cstring>
#include <exception>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include "common/noncopyable.h"
namespace worldview {
namespace Sqlite {
class Stmt;

class Conn : public noncopyable {
 public:
  using Ptr = std::shared_ptr<Conn>;
  Conn(const std::string &db_name, int flags = SQLITE_OPEN_READONLY)
      : filename_(db_name) {
    if (SQLITE_OK != sqlite3_open_v2(db_name.c_str(), &db_, flags, NULL)) {
      std::string err(sqlite3_errmsg(db_));
      sqlite3_close(db_);
      db_ = nullptr;
      throw std::runtime_error(err);
    }
  }

  const std::string &filename() const { return filename_; }

  ~Conn() {
    if (db_) sqlite3_close(db_);
  }

 private:
  sqlite3 *db_ = nullptr;
  std::string filename_;
  friend class Stmt;
  friend class Transaction;
};

class Stmt : public noncopyable {
 public:
  using Ptr = std::shared_ptr<Stmt>;
  Stmt(const Conn::Ptr &conn, const std::string &stmt_str)
      : conn_(conn), stmt_str_(stmt_str) {
    // sqlite3 requires length including terminating 0
    if (SQLITE_OK != sqlite3_prepare_v2(conn_->db_, stmt_str.c_str(),
                                        stmt_str.length() + 1, &stmt_, NULL)) {
      throw std::runtime_error(sqlite3_errmsg(conn_->db_));
    }
  }
  virtual ~Stmt() { sqlite3_finalize(stmt_); }
  int step() {
    int rc = sqlite3_step(stmt_);
    if (SQLITE_DONE != rc && SQLITE_ROW != rc) {
      throw std::runtime_error(sqlite3_errmsg(conn_->db_));
    }
    return rc;
  }
  void reset() {
    if (SQLITE_OK != sqlite3_clear_bindings(stmt_)) {
      throw std::runtime_error(sqlite3_errmsg(conn_->db_));
    }
    if (SQLITE_OK != sqlite3_reset(stmt_)) {
      throw std::runtime_error(sqlite3_errmsg(conn_->db_));
    }
  }

  int bind(int idx, double value) {
    return sqlite3_bind_double(stmt_, idx, value);
  }
  int bind(int idx, int value) { return sqlite3_bind_int(stmt_, idx, value); }
  int bind(int idx, int64_t value) {
    return sqlite3_bind_int64(stmt_, idx, value);
  }
  int bind(int idx, const std::string &value) {
    return sqlite3_bind_text(stmt_, idx, value.c_str(), value.length(),
                             SQLITE_TRANSIENT);
  }
  int bind(int idx, const void *value, int n) {
    return sqlite3_bind_blob(stmt_, idx, value, n, SQLITE_TRANSIENT);
  }
  int bind(int idx) { return sqlite3_bind_null(stmt_, idx); }

  int changes() { return sqlite3_changes(conn_->db_); }

  const std::string &toString() const { return stmt_str_; }

 protected:
  sqlite3_stmt *stmt_ = nullptr;
  Conn::Ptr conn_;
  std::string stmt_str_;
};

class Query : public Stmt {
 public:
  using Stmt::Stmt;

  int column_count() const { return sqlite3_column_count(stmt_); }
  const char *column_name(int idx) const {
    return sqlite3_column_name(stmt_, idx);
  }
  const char *column_decltype(int idx) const {
    return sqlite3_column_decltype(stmt_, idx);
  }

 private:
  class Row {
   public:
    using Ptr = std::shared_ptr<Row>;

    Row(Query *q) : q_(q) {}
    int data_count() const { return sqlite3_data_count(q_->stmt_); }
    // INTEGER = 1 FLOAT TEXT BLOB NULL
    int column_type(int idx) const {
      return sqlite3_column_type(q_->stmt_, idx);
    }
    const char *column_type_str(int idx) const {
      switch (column_type(idx)) {
#define TYPE_CASE(NAME) \
  case NAME:            \
    return #NAME
        TYPE_CASE(SQLITE_INTEGER);
        TYPE_CASE(SQLITE_FLOAT);
        TYPE_CASE(SQLITE_TEXT);
        TYPE_CASE(SQLITE_BLOB);
        TYPE_CASE(SQLITE_NULL);
        default:
          return "UNDEFINED";
      }
    }
    int column_bytes(int idx) const {
      return sqlite3_column_bytes(q_->stmt_, idx);
    }

    template <typename T>
    inline T get(int idx) const;

   private:
    Query *q_;
  };

 public:
  class iterator : public std::iterator<std::input_iterator_tag, Row> {
   public:
    iterator(Query *q = nullptr) : q_(q) {
      if (q_) {
        last_rc_ = q_->step();
        if (SQLITE_ROW == last_rc_) {
          r_ = std::make_shared<Row>(q);
        }
      }
    }
    bool operator==(const iterator &rhs) const {
      return last_rc_ == rhs.last_rc_;
    }
    bool operator!=(const iterator &rhs) const {
      return last_rc_ != rhs.last_rc_;
    }
    iterator &operator++() {
      last_rc_ = q_->step();
      return *this;
    }
    reference operator*() const { return *r_; }
    pointer operator->() const { return r_.get(); }

   private:
    Query *q_ = nullptr;
    Row::Ptr r_ = nullptr;
    int last_rc_ = SQLITE_DONE;
  };

  iterator begin() { return iterator(this); }
  iterator end() { return iterator(); }
};

template <>
inline int Query::Row::get<int>(int idx) const {
  return sqlite3_column_int(q_->stmt_, idx);
}

template <>
inline double Query::Row::get<double>(int idx) const {
  return sqlite3_column_double(q_->stmt_, idx);
}

template <>
inline int64_t Query::Row::get<int64_t>(int idx) const {
  return sqlite3_column_int64(q_->stmt_, idx);
}

template <>
inline const char *Query::Row::get<const char *>(int idx) const {
  return reinterpret_cast<const char *>(sqlite3_column_text(q_->stmt_, idx));
}

template <>
inline const void *Query::Row::get<const void *>(int idx) const {
  return sqlite3_column_blob(q_->stmt_, idx);
}

template <>
inline std::string Query::Row::get<std::string>(int idx) const {
  return std::string(get<const char *>(idx),
                     sqlite3_column_bytes(q_->stmt_, idx));
}

template <>
inline std::vector<char> Query::Row::get<std::vector<char>>(int idx) const {
  const char *data = reinterpret_cast<const char *>(get<const void *>(idx));
  int n = sqlite3_column_bytes(q_->stmt_, idx);

  std::vector<char> res(n);
  memcpy(res.data(), data, n);
  return res;
}

class Transaction : public noncopyable {
 public:
  explicit Transaction(const Conn::Ptr &conn) : conn_(conn) {
    sqlite3_exec(conn_->db_, "begin;", 0, 0, 0);
  }

  ~Transaction() { sqlite3_exec(conn_->db_, "commit;", 0, 0, 0); }

 protected:
  Conn::Ptr conn_;
};

}  // namespace Sqlite
}  // namespace worldview