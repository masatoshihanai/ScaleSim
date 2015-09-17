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

//#include <boost/filesystem.hpp>
//#include <boost/thread.hpp>
//#include <boost/shared_ptr.hpp>
//#include <boost/make_shared.hpp>
//#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/binary_oarchive.hpp>
//
//template<class App>
//class berkeley_db_store {
// private:
//  berkeley_db_store(const berkeley_db_store&);
//  void operator=(const berkeley_db_store&);
// public:
//  berkeley_db_store(std::string object_type, long rank) {
//    std::stringstream ss;
//    ss << rank;
//    env = new DbEnv(0);
//// todo   env_name = DB_TMP_HOME + object_type + ss.str();
//    std::string command_mkdir = "mkdir " + env_name;
//    system(command_mkdir.c_str());
//
//    u_int32_t env_flags = DB_CREATE     |  // If the environment does not exist, create it.
//                          DB_INIT_LOCK  |  // Initialize locking
//                          DB_INIT_LOG   |  // Initialize logging
//                          DB_INIT_MPOOL |  // Initialize the cache
//                          DB_INIT_TXN;     // Initialize transactions
//    env->open(env_name.c_str(), env_flags, 0);
//
//    db = new Db(env, 0);
//    u_int32_t oFlags = DB_CREATE     |
//                       DB_AUTO_COMMIT;
//    db->open(NULL,                // Transaction pointer
//             "database.db",       // Database file name
//             NULL,                // Optional logical database name
//             DB_BTREE,            // Database access method
//             oFlags,              // Open flags
//             0);                  // File mode (using defaults)
//  };
//  virtual ~berkeley_db_store() {
//    db->close(0);
//    env->close(0);
//    delete env;
//    delete db;
//    std::string command_rm_env = "rm -r " + env_name;
//    system(command_rm_env.c_str());
//  }
// private:
//  std::string db_file_name;
//  std::string env_name;
//  int binary_size;
//  Db* db;
//  DbEnv* env;
//  boost::mutex put_mutex_;
//  boost::mutex get_mutex_;
//  std::map<long, long> min_loaded_time_; // <lp's id, mim_loaded_time>
// private:
//  std::vector<char> convert_key_id_to_key_char(long key, long id_) {
//    std::vector<char> ret(40, '0');
//    ret[39] = '\0';
//    std::stringstream ss;
//    ss << key;
//    std::string key_str = ss.str();
//    for (int i = 0; i < key_str.size(); ++i) {
//      ret[38 - i] = key_str[key_str.size() - 1 - i];
//    }
//    ret[19] = '\0';
//    std::stringstream ss_;
//    ss_ << id_;
//    std::string id_value = ss_.str();
//    for (int i = 0; i < id_value.size(); ++i) {
//      ret[18 - i] = id_value[id_value.size() - 1 - i];
//    }
//    return ret;
//  };
//  std::pair<long, long> convert_db_key_to_id_key(char* db_key) {
//    char key_char[20];
//    char id_char[20];
//    for (int i = 0; i < 20; ++i) {
//      id_char[i] = db_key[i];
//    }
//    for (int i = 0; i < 20; ++i) {
//      key_char[i] = db_key[i + 20];
//    }
//    return std::pair<long, long>(atol(id_char), atol(key_char));
//  };
// public:
//  void put(long key, App value, long id) {
//    boost::lock_guard<boost::mutex> guard(put_mutex_);
//    if (min_loaded_time_.find(id) == min_loaded_time_.end()) {
//      min_loaded_time_.insert(std::pair<long, long>(id, std::numeric_limits<long>::max()));
//    }
//    std::stringstream ss;
//    boost::archive::binary_oarchive ar(ss);
//    ar << value;
//    std::string binary = ss.str();
//    std::vector<char> put_event_binary(binary.begin(), binary.end());
//    put_event_binary.push_back('\0');
//
//    std::vector<char> key_char
//      = convert_key_id_to_key_char(key, id);
//    Dbt db_key(&key_char[0], key_char.size());
//    binary_size = put_event_binary.size() + 1;
//    Dbt data(&put_event_binary[0], binary_size);
//    DbTxn* txn = NULL;
//    env->txn_begin(NULL, &txn, 0);
//    int ret = db->put(txn, &db_key, &data, 0);
//    txn->commit(0);
//  };
//  boost::shared_ptr<App> get(long key, long id) {
//    boost::lock_guard<boost::mutex> guard(get_mutex_);
//    std::vector<char> key_char = convert_key_id_to_key_char(key, id);
//    Dbt db_key(&key_char[0], key_char.size());
//    char* get_event_binary;
//    Dbt get_data(get_event_binary, binary_size);
//
//    db->get(NULL, &db_key, &get_data, 0);
//
//    std::string get_data_binary
//      = std::string((char*) get_data.get_data(), binary_size);
//    std::stringstream from_ss;
//    from_ss << get_data_binary;
//    boost::archive::binary_iarchive iar(from_ss);
//    App obj;
//    iar >> obj;
//    boost::shared_ptr<App> ret = boost::make_shared<App>(obj);
//    return ret;
//  };
//  boost::shared_ptr<std::vector<std::pair<long, boost::shared_ptr<const App> > > >
//  get_range(long from, long id) {
//    boost::lock_guard<boost::mutex> guard(get_mutex_);
//    if (min_loaded_time_.find(id) == min_loaded_time_.end()) {
//      min_loaded_time_.insert(std::pair<long, long>(id, std::numeric_limits<long>::max()));
//    }
//    typedef boost::shared_ptr<const App> obj_ptr;
//    typedef boost::shared_ptr<std::vector<std::pair<long, obj_ptr > > > pairs_ptr;
//    pairs_ptr ret_pairs = pairs_ptr(new std::vector<std::pair<long, obj_ptr> >);
//    long to = min_loaded_time_[id];
//    if (from > to) {
//      return ret_pairs;
//    }
//    Dbc* cursorp;
//    db->cursor(NULL, &cursorp, 0);
//    std::vector<char> from_char = convert_key_id_to_key_char(from, id);
//    Dbt db_key = Dbt(&from_char[0], from_char.size());
//    char* get_event_binary;
//    Dbt data(get_event_binary, binary_size);
//    int ret;
//
//    // get cursor of "from"
//    ret = cursorp->get(&db_key, &data, DB_SET_RANGE);
//    if (ret) {
//      return ret_pairs;
//    }
//    std::string get_data_binary
//      = std::string((char*) data.get_data(), binary_size);
//    std::stringstream from_ss;
//    from_ss << get_data_binary;
//    boost::archive::binary_iarchive iar(from_ss);
//    App deserialize_obj;
//    iar >> deserialize_obj;
//    boost::shared_ptr<App> obj = boost::make_shared<App>(deserialize_obj);
//    std::pair<long, long> id_key = convert_db_key_to_id_key((char*) db_key.get_data());
//    ret_pairs->push_back(std::pair<long, obj_ptr>(id_key.second, obj));
//    // get until end
//    while ((ret = cursorp->get(&db_key, &data, DB_NEXT)) == 0) {
//      std::pair<long, long> id_key = convert_db_key_to_id_key((char*) db_key.get_data());
//      if (id_key.second >= to || id_key.first != id) {
//        break;
//      }
//      get_data_binary = std::string((char*) data.get_data(), binary_size);
//      std::stringstream ss;
//      ss << get_data_binary;
//      boost::archive::binary_iarchive iar(ss);
//      App deserialize_obj_loop;
//      iar >> deserialize_obj_loop;
//      boost::shared_ptr<App> new_obj = boost::make_shared<App>(deserialize_obj_loop);
//      ret_pairs->push_back(std::pair<long, obj_ptr>(id_key.second, new_obj));
//    }
//    if (cursorp != NULL) cursorp->close();
//    min_loaded_time_[id] = from;
//    return ret_pairs;
//  };
//  void reset_min_loaded_time(long id) {
//    if (min_loaded_time_.find(id) == min_loaded_time_.end()) {
//      return;
//    }
//    min_loaded_time_[id] = std::numeric_limits<long>::max();
//  };
//};
//
#endif /* SCALESIM_LOGICAL_PROCESS_STORE_BERKELEYDB_STORE_HPP_ */
