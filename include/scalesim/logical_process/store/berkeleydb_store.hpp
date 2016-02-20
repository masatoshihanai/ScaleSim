/*
 * berkeleydb_store.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_LOGICAL_PROCESS_STORE_BERKELEYDB_STORE_HPP_
#define SCALESIM_LOGICAL_PROCESS_STORE_BERKELEYDB_STORE_HPP_

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <gflags/gflags.h>

#include <db_cxx.h>

#include "scalesim/util.hpp"

namespace scalesim {

template<class Object>
class berkeleydb_store {
  typedef boost::shared_ptr<const Object> obj_ptr;
 private:
  berkeleydb_store(const berkeleydb_store&);
  void operator=(const berkeleydb_store&);
 public:
  berkeleydb_store() {};
  virtual ~berkeleydb_store() {};

 private:
  Db* db;
  DbEnv* env;
  boost::mutex put_mutex_;
  boost::mutex get_mutex_;

 public:
  void init(const std::string& object_type, const int rank);
  void init_read(const std::string& object_type, const int rank);
  void finish();
  void clear_db();
  void put(const timestamp& key, const long lp_id, const Object& value);
  void put_range(const std::vector<timestamp>& keys,
                 const std::vector<obj_ptr>& values,
                 const long lp_id);
  void get(const timestamp& key, const long lp_id, obj_ptr& ret);
  void get_prev(const timestamp& key, const long lp_id,
                obj_ptr& ret, timestamp& time_of_ret);
  void get_range(const timestamp& from, const timestamp& to,
                 const long lp_id, std::vector<obj_ptr >& ret_obj);

 private:
  void key_lpid_to_char(const timestamp& key, const long lp_id,
                        std::vector<char>& ret);
  void char_to_key_lpid(const char* const db_key,
                        timestamp& tmstmp, long& lp_id);
  void delete_directory(boost::filesystem::path dir);
};

template<class Object>
void berkeleydb_store<Object>::init(const std::string& type, const int rank) {
  DLOG_ASSERT(!FLAGS_store_dir.empty())
      << "FLAG failure. You have to set directory path from commandline. "
      << " Run with --help,"
      << " and see how to run Differential Execution with file systems.";
  std::string rank_str;
  std::stringstream ss_rank;
  ss_rank << rank;
  std::string id_path_str_ = FLAGS_store_dir + "/" + FLAGS_sim_id;
  std::string db_path_str_ = id_path_str_ + "/" + type + ss_rank.str();

  /* make directory */
  boost::filesystem::path id_path_(id_path_str_);
  boost::filesystem::path db_path_(db_path_str_);
  boost::filesystem::create_directory(FLAGS_store_dir);
  boost::filesystem::create_directory(id_path_);
  boost::filesystem::create_directory(db_path_);
  LOG_IF(INFO, rank == 0) << "Create directory: " << id_path_str_;

  /* open database */
  env = new DbEnv(0);
  u_int32_t env_flags = DB_CREATE     |  // If the environment does not exist, create it.
                        DB_INIT_LOCK  |  // Initialize locking
                        DB_INIT_LOG   |  // Initialize logging
                        DB_INIT_MPOOL |  // Initialize the cache
                        DB_INIT_TXN;     // Initialize transactions
  env->open(db_path_str_.c_str(), env_flags, 0);
  db = new Db(env, 0);
  u_int32_t oFlags = DB_CREATE     |
                     DB_AUTO_COMMIT;
  db->open(NULL,                // Transaction pointer
           "database.db",       // Database file name
           NULL,                // Optional logical database name
           DB_BTREE,            // Database access method
           oFlags,              // Open flags
           0);                  // File mode (using defaults)
};

template<class Object>
void berkeleydb_store<Object>::init_read(const std::string& type, const int rank) {
  DLOG_ASSERT(!FLAGS_store_dir.empty())
      << "FLAGS failure. You have to set directory path from commandline. "
      << " Run with --help, "
      << " and see how to run Differential Execution with file system.";
  std::string rank_str;
  std::stringstream ss_rank;
  ss_rank << rank;
  std::string id_path_str_ = FLAGS_store_dir + "/" + FLAGS_sim_id;

  /* check directory */
  boost::filesystem::path id_path_(id_path_str_);
  DLOG_ASSERT(boost::filesystem::exists(id_path_))
      << " No directory: " << id_path_str_
      << " Check sim_id and database directory.";

  /* open database */
  env = new DbEnv(0);
  std::string db_path_str_ = id_path_str_ + "/" + type + ss_rank.str();
  u_int32_t env_flags = DB_CREATE     |  // If the environment does not exist, create it.
                        DB_INIT_LOCK  |  // Initialize locking
                        DB_INIT_LOG   |  // Initialize logging
                        DB_INIT_MPOOL |  // Initialize the cache
                        DB_INIT_TXN;     // Initialize transactions
  env->open(db_path_str_.c_str(), env_flags, 0);
  db = new Db(env, 0);
  u_int32_t oFlags = DB_CREATE     |
                     DB_AUTO_COMMIT;
  db->open(NULL,                // Transaction pointer
           "database.db",       // Database file name
           NULL,                // Optional logical database name
           DB_BTREE,            // Database access method
           oFlags,              // Open flags
           0);                  // File mode (using defaults)
};

template<class Object>
void berkeleydb_store<Object>::finish() {
  if (db) return;
  try {
    db->close(0);
    env->close(0);
    delete db;
    delete env;
    db = NULL;
    env = NULL;
  } catch (std::exception &e) {
    std::cerr << "db finish failure" << std::endl;
    std::exit(2);
  }
};

template<class Object>
void berkeleydb_store<Object>::clear_db() {
  /* delete db */
  if (!db) {
    try {
      db->close(0);
      env->close(0);
      delete db;
      delete env;
      db = NULL;
      env = NULL;
    } catch (std::exception &e) {
      std::cerr << "db finish failure" << std::endl;
      std::exit(2);
    }
  }

  /* delete files */
  boost::filesystem::path dir(FLAGS_store_dir);
  delete_directory(dir);
};

template<class Object>
void berkeleydb_store<Object>::put(const timestamp& time,
                                   const long lp_id,
                                   const Object& value) {
  boost::lock_guard<boost::mutex> guard(put_mutex_);
  /* generate key */
  std::vector<char> key;
  key_lpid_to_char(time, lp_id, key);

  /* serialize object */
  std::stringstream ss;
  boost::archive::binary_oarchive ar(ss);
  ar << value;
  std::string binary = ss.str();
  std::vector<char> put_event_binary(binary.begin(), binary.end());

  Dbt db_key(&key[0], key.size());
  Dbt data(&put_event_binary[0], put_event_binary.size());
  DbTxn* txn = NULL;
  env->txn_begin(NULL, &txn, 0);
  int ret = db->put(txn, &db_key, &data, 0);
  txn->commit(0);
};

template<class Object>
void berkeleydb_store<Object>::put_range(const std::vector<timestamp>& keys,
                                         const std::vector<obj_ptr>& values,
                                         const long lp_id) {
  boost::lock_guard<boost::mutex> guard(put_mutex_);
  for (int i = 0; i < keys.size(); ++i) {
    /* generate key */
    std::vector<char> key;
    key_lpid_to_char(keys[i], lp_id, key);

    /* serialize object */
    std::stringstream ss;
    boost::archive::binary_oarchive ar(ss);
    ar << *(values[i]);
    std::string binary = ss.str();
    std::vector<char> put_event_binary(binary.begin(), binary.end());

    Dbt db_key(&key[0], key.size());
    Dbt data(&put_event_binary[0], put_event_binary.size());
    DbTxn* txn = NULL;
    env->txn_begin(NULL, &txn, 0);
    int ret = db->put(txn, &db_key, &data, 0);
    txn->commit(0);
  }
};

template<class Object>
void berkeleydb_store<Object>::get(const timestamp& time,
                                   const long lp_id,
                                   obj_ptr& ret) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  /* generate key */
  std::vector<char> key;
  key_lpid_to_char(time, lp_id, key);
  Dbt db_key(&key[0], key.size());

  /* get data */
  char* event_binary;
  Dbt data;
  data.set_data(event_binary);
  data.set_flags(DB_DBT_MALLOC);

  db->get(NULL, &db_key, &data, 0);

  /* deserialize the data */
  std::string binary = std::string((char*) data.get_data(), data.get_size());
  std::stringstream from_ss;
  from_ss << binary;
  boost::archive::binary_iarchive iar(from_ss);
  Object obj;
  iar >> obj;

  ret = boost::make_shared<Object>(obj);
};

template<class Object>
void berkeleydb_store<Object>::get_prev(const timestamp& time,
                                        const long lp_id,
                                        obj_ptr& ret_obj,
                                        timestamp& time_of_ret) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  /* generate key */
  std::vector<char> key;
  key_lpid_to_char(time, lp_id, key);
  Dbt db_key = Dbt(&key[0], key.size());

  /* prepare data */
  char* binary;
  Dbt data;
  data.set_data(binary);
  data.set_flags(DB_DBT_MALLOC);

  /* seek to key */
  Dbc* cursorp;
  db->cursor(NULL, &cursorp, 0);
  int ret = cursorp->get(&db_key, &data, DB_SET_RANGE);

  /* seek back */
  ret = cursorp->get(&db_key, &data, DB_PREV);

  /* get value */
  std::string obj_binary
      = std::string((char*) data.get_data(), data.get_size());
  std::stringstream ss;
  ss << obj_binary;
  boost::archive::binary_iarchive iar(ss);
  Object obj;
  iar >> obj;
  ret_obj = boost::make_shared<Object>(obj);

  /* get key */
  long lp_id_;
  char_to_key_lpid((char*) db_key.get_data(), time_of_ret, lp_id_);
  if (cursorp != NULL) cursorp->close();
};

template<class Object>
void berkeleydb_store<Object>::get_range(const timestamp& from,
                                         const timestamp& to,
                                         const long lp_id,
                                         std::vector<obj_ptr>& ret_obj) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  if (from > to) return;

  /* generate key */
  std::vector<char> from_char;
  key_lpid_to_char(from, lp_id, from_char);
  Dbt db_key = Dbt(&from_char[0], from_char.size());

  /* prepare data */
  char* binary;
  Dbt data;
  data.set_data(binary);
  data.set_flags(DB_DBT_MALLOC);

  /* seek to from */
  Dbc* cursorp;
  db->cursor(NULL, &cursorp, 0);
  int ret = cursorp->get(&db_key, &data, DB_SET_RANGE);
  if (ret) {
    if (cursorp != NULL) cursorp->close();
    return;
  }
  std::string get_data_binary
      = std::string((char*) data.get_data(), data.get_size());
  std::stringstream from_ss;
  from_ss << get_data_binary;
  boost::archive::binary_iarchive iar(from_ss);
  Object obj;
  iar >> obj;
  ret_obj.push_back(boost::make_shared<Object>(obj));

  /* get until end */
  while ((ret = cursorp->get(&db_key, &data, DB_NEXT)) == 0) {
    timestamp tmstmp;
    long id;
    char_to_key_lpid((char*) db_key.get_data(), tmstmp, id);
    if (tmstmp > to || id != lp_id) break;

    get_data_binary = std::string((char*) data.get_data(), data.get_size());
    std::stringstream ss;
    ss << get_data_binary;
    boost::archive::binary_iarchive iar(ss);
    Object deserialized_obj;
    iar >> deserialized_obj;
    ret_obj.push_back(boost::make_shared<Object>(deserialized_obj));
  }

  if (cursorp != NULL) cursorp->close();
};

/*
 * convert from (time stamp + lp id) to char[60]
 * ex)
 *  from:
 *   time stamp = (time = 99,999,999,999, obj id = 10)
 *   lp id      = 555
 *  to:
 *   0000000000 000000555'\0' 0000000099 999999999'\0' 000000000 000000010'\0'
 */
template<class Object>
void berkeleydb_store<Object>::key_lpid_to_char(const timestamp& tmstmp,
                                             const long lp_id_,
                                             std::vector<char>& ret) {
  ret = std::vector<char>(60, '0');

  /* convert object id to char[20] */
  ret[59] = '\0';
  std::stringstream obj_id_ss_;
  obj_id_ss_ << tmstmp.id();
  std::string obj_id_str_ = obj_id_ss_.str();
  for (int i = 0; i < obj_id_str_.size(); ++i) {
    ret[58 - i] = obj_id_str_[obj_id_str_.size() - 1 - i];
  }

  /* convert time to char[20] */
  ret[39] = '\0';
  std::stringstream time_ss_;
  time_ss_ << tmstmp.time();
  std::string time_str = time_ss_.str();
  for (int i = 0; i < time_str.size(); ++i) {
    ret[38 - i] = time_str[time_str.size() - 1 - i];
  }

  /* convert lp id to char[20] */
  ret[19] = '\0';
  std::stringstream lp_id_ss_;
  lp_id_ss_ << lp_id_;
  std::string lp_id_str_ = lp_id_ss_.str();
  for (int i = 0; i < lp_id_str_.size(); ++i) {
    ret[18 - i] = lp_id_str_[lp_id_str_.size() - 1 - i];
  }
};

/*
 * convert from char[60] to (time stamp + lp id)
 * ex)
 *  from:
 *   0000000000 0000000555 0000000009 9999999999 000000000 0000000010
 *  to:
 *   time stamp = (time = 99,999,999,999, obj id = 10)
 *   lp id      = 555
 */
template<class Object>
void berkeleydb_store<Object>::char_to_key_lpid(const char* const db_key,
                                             timestamp& tmstmp,
                                             long& lp_id) {
  /* convert lpid */
  char lpid_char[20];
  for (int i = 0; i < 20; ++i) {
    lpid_char[i] = db_key[i];
  }
  lp_id = atol(lpid_char);

  /* convert time */
  char time_char[20];
  for (int i = 0; i < 20; ++i) {
    time_char[i] = db_key[i + 20];
  }
  long time_ = atol(time_char);

  /* convert obj id */
  char obj_id_char[20];
  for (int i = 0; i < 20; ++i) {
    obj_id_char[i] = db_key[i + 40];
  }
  long obj_id_ = atol(obj_id_char);

  tmstmp = timestamp(time_, obj_id_);
};

template<class Object>
void berkeleydb_store<Object>::delete_directory(boost::filesystem::path dir) {
  if (!boost::filesystem::exists(dir)) return;

  boost::filesystem::directory_iterator end;
  for (boost::filesystem::directory_iterator p(dir);
      p != end; ++p) {
    if (boost::filesystem::is_directory(*p)) {
      delete_directory(*p);
    } else {
      boost::filesystem::remove(*p);
    }
  }
  boost::filesystem::remove(dir);
};

} /* namespace scalesim */

#endif /* SCALESIM_LOGICAL_PROCESS_STORE_BERKELEYDB_STORE_HPP_ */
