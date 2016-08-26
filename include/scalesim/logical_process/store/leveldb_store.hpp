/*
 * leveldb_store.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_LOGICAL_PROCESS_STORE_LEVELDB_STORE_HPP_
#define SCALESIM_LOGICAL_PROCESS_STORE_LEVELDB_STORE_HPP_

#include <unistd.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <gflags/gflags.h>
#include "leveldb/db.h"
#include "leveldb/slice.h"
#include "leveldb/options.h"
#include "leveldb/status.h"
#include "leveldb/write_batch.h"
#include "scalesim/util.hpp"

namespace scalesim {

/* Memory ratio from all system memory for write-buffer */
static const double WRITE_BUF_MEM_RATIO = 0.1; /* 10% of system memory for buffer */

template<class Object>
class leveldb_store {
  typedef boost::shared_ptr<const Object> obj_ptr;
 private:
   leveldb_store(const leveldb_store&);
   void operator=(const leveldb_store&);
 public:
  leveldb_store() {};
  virtual ~leveldb_store() {};

 private:
  leveldb::DB* db;
  leveldb::WriteBatch* batch;
  std::string db_path_str_;
  std::string type_;
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
void leveldb_store<Object>::init(const std::string& obj_type, const int rank) {
  DLOG_ASSERT(!FLAGS_store_dir.empty())
      << " FLAGS failure. You have to set directory path from commandline. "
      << " Run with --help,"
      << " and see how to run Differential Execution with file system.";
  std::stringstream ss_rank;
  ss_rank << rank;
  std::string id_path_str_ = FLAGS_store_dir + "/" + FLAGS_sim_id;

  /* make directory */
  boost::filesystem::path id_path_(id_path_str_);
  boost::filesystem::create_directory(FLAGS_store_dir);
  boost::filesystem::create_directory(id_path_);
  LOG_IF(INFO, rank == 0) << "Create directory: " << id_path_str_;

  /* open database */
  leveldb::Options options;
  options.create_if_missing = true;
  options.error_if_exists = true;
  options.write_buffer_size
      = (size_t)(sysconf(_SC_PHYS_PAGES) * sysconf(_SC_PAGE_SIZE) * WRITE_BUF_MEM_RATIO);
  type_ = obj_type;
  db_path_str_ = id_path_str_ + "/" + type_ + ss_rank.str();
  leveldb::Status status = leveldb::DB::Open(options, db_path_str_, &db);
  DLOG_ASSERT(status.ok()) << status.ToString();
  LOG(INFO) << "Open database: " << db_path_str_;
  batch = new leveldb::WriteBatch();
};

template<class Object>
void leveldb_store<Object>::init_read(const std::string& type, const int rank) {
  DLOG_ASSERT(!FLAGS_store_dir.empty())
      << " FLAGS failure. You have to set directory path from commandline. "
      << " Run with --help,"
      << " and see how to run Differential Execution with file system.";
  std::stringstream ss_rank;
  ss_rank << rank;
  std::string id_path_str_ = FLAGS_store_dir + "/" + FLAGS_sim_id;

  /* check directory */
  boost::filesystem::path id_path_(id_path_str_);
  DLOG_ASSERT(boost::filesystem::exists(id_path_))
      << " No directory: " << id_path_str_
      << " Check  sim_id and database directory.";

  /* open data base */
  // todo open multiple process in read only mode
  leveldb::Options options;
  options.create_if_missing = false;
  options.error_if_exists = false;
  std::string db_path_str_ = id_path_str_ + "/" + type + ss_rank.str();
  leveldb::Status status = leveldb::DB::Open(options, db_path_str_, &db);
  DLOG_ASSERT(status.ok()) << status.ToString();
  LOG(INFO) << "Open database: " << db_path_str_;
  batch = NULL;
}

template<class Object>
void leveldb_store<Object>::finish() {
  if (batch) {
    stopwatch::instance("DBWrite")->start();
    leveldb::WriteOptions option;
    option.sync = true;
    db->Write(option, batch);
    delete batch;
    batch = NULL;
    stopwatch::instance("DBWrite")->stop();
    /* Check DB size */
    int size = 0;
    for (boost::filesystem::recursive_directory_iterator it(db_path_str_);
         it != boost::filesystem::recursive_directory_iterator(); ++it) {
      if(!boost::filesystem::is_directory(*it)) {
        size += boost::filesystem::file_size(*it);
      }
    }
    *scalesim::counter::instance("FileSize::" + type_) += size;
  }
  delete db;
  db = NULL;
};

template<class Object>
void leveldb_store<Object>::clear_db() {
  /* delete db */
  if (db) { delete db; db = NULL; }

  /* delete files */
  boost::filesystem::path dir(FLAGS_store_dir);
  delete_directory(dir);
};

template<class Object>
void leveldb_store<Object>::put(const timestamp& time, const long id,
                                const Object& value) {
  /* generate key */
  std::vector<char> key;
  key_lpid_to_char(time, id, key);

  /* serialize object */
  std::stringstream ss;
  boost::archive::binary_oarchive ar(ss);
  ar << value;
  std::string obj_binary = ss.str();

  /* put object */
  leveldb::Slice db_key((char *) &key[0], key.size());
  {
    boost::lock_guard<boost::mutex> guard(put_mutex_);
    batch->Put(db_key, obj_binary);
  }
};

template<class Object>
void leveldb_store<Object>::put_range(const std::vector<timestamp>& keys,
                                      const std::vector<obj_ptr>& values,
                                      const long id) {
  boost::lock_guard<boost::mutex> guard(put_mutex_);
  { /* batch write */
    for (int i = 0; i < keys.size(); ++i) {
      /* generate key */
      std::vector<char> key;
      key_lpid_to_char(keys[i], id, key);

      /* serialize object */
      std::stringstream ss;
      boost::archive::binary_oarchive ar(ss);
      ar << *(values[i]);
      std::string obj_binary = ss.str();

      /* put object */
      leveldb::Slice db_key((char*) &key[0], key.size());
      batch->Put(db_key, obj_binary);
    }
  }
};

template<class Object>
void leveldb_store<Object>::get(const timestamp& time, const long lp_id,
                                obj_ptr& ret) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  /* For TEST */
  if (batch) {
    db->Write(leveldb::WriteOptions(), batch);
    delete batch;
    batch = new leveldb::WriteBatch();
  }

  /* generate key */
  std::vector<char> key;
  key_lpid_to_char(time, lp_id, key);
  leveldb::Slice db_key((char*) &key[0], key.size());

  /* get value */
  std::string ret_binary;
  leveldb::Status s = db->Get(leveldb::ReadOptions(), db_key, &ret_binary);

  /* deserialize value */
  std::stringstream ss;
  ss << ret_binary;
  boost::archive::binary_iarchive iar(ss);
  Object deserialized_obj;
  iar >> deserialized_obj;
  ret = boost::make_shared<Object>(deserialized_obj);
};

template<class Object>
void leveldb_store<Object>::get_prev(const timestamp& time,
                                     const long lp_id,
                                     obj_ptr& ret,
                                     timestamp& time_of_ret) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  /* For TEST */
  if (batch) {
    db->Write(leveldb::WriteOptions(), batch);
    delete batch;
    batch = new leveldb::WriteBatch();
  }

  /* generate key */
  std::vector<char> key;
  key_lpid_to_char(time, lp_id, key);
  leveldb::Slice db_key((char*) &key[0], key.size());
  /* seek */
  leveldb::Iterator* it = db->NewIterator(leveldb::ReadOptions());

  it->Seek(db_key);

  if (!it->Valid()) { it->SeekToLast(); }

  it->Prev();

  if (!it->Valid()) { it->SeekToFirst(); }
  if (!it->Valid()) { return; }

  /* get value */
  std::string obj_binary = it->value().ToString();

  /* deserialize object */
  std::stringstream ss;
  ss << obj_binary;
  boost::archive::binary_iarchive iar(ss);
  Object deserialized_obj;
  iar >> deserialized_obj;
  ret = boost::make_shared<Object>(deserialized_obj);

  /* get key */
  long lp_id_;
  char_to_key_lpid(it->key().data(), time_of_ret, lp_id_);
};

template<class Object>
void leveldb_store<Object>::get_range(const timestamp& from,
                                      const timestamp& to,
                                      const long lp_id,
                                      std::vector<obj_ptr>& ret_obj) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  if (from == to || from > to) return;
  /* For TEST */
  if (batch) {
    db->Write(leveldb::WriteOptions(), batch);
    delete batch;
    batch = new leveldb::WriteBatch();
  }

  /* generate key */
  std::vector<char> key_from_;
  key_lpid_to_char(from, lp_id, key_from_);
  leveldb::Slice db_key_from_((char*) &key_from_[0], key_from_.size());

  std::vector<char> key_to_;
  key_lpid_to_char(to, lp_id, key_to_);
  leveldb::Slice db_key_to_((char*) &key_to_[0], key_to_.size());
  /* seek and get from database */
  leveldb::Iterator* it = db->NewIterator(leveldb::ReadOptions());
  for (it->Seek(db_key_from_); it->Valid(); it->Next()) {
    if (it->key().compare(db_key_to_) >= 0) { break; }

    /* get value */
    std::string obj_binary = it->value().ToString();

    /* deserialize object */
    std::stringstream ss;
    ss << obj_binary;
    boost::archive::binary_iarchive iar(ss);
    Object deserialized_obj;
    iar >> deserialized_obj;
    obj_ptr obj = boost::make_shared<Object>(deserialized_obj);
    ret_obj.push_back(obj);
  }
  delete it;
};

/*
 * convert from (time stamp + lp id) to char[60]
 * ex)
 *  from:
 *   time stamp = (time = 99,999,999,999, obj id = 10)
 *   lp id      = 555
 *  to:
 *   0000000000 0000000555 0000000009 9999999999 000000000 0000000010
 */
template<class Object>
void leveldb_store<Object>::key_lpid_to_char(const timestamp& tmstmp,
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
void leveldb_store<Object>::char_to_key_lpid(const char* const db_key,
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
void leveldb_store<Object>::delete_directory(boost::filesystem::path dir) {
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

#endif /* SCALESIM_LOGICAL_PROCESS_STORE_LEVELDB_STORE_HPP_ */
