/*
 * redis_store.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_LOGICAL_PROCESS_STORE_REDIS_STORE_HPP_
#define SCALESIM_LOGICAL_PROCESS_STORE_REDIS_STORE_HPP_

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <hiredis/hiredis.h>

namespace scalesim {

template<class Object>
class redis_store {
  typedef boost::shared_ptr<const Object> obj_ptr;
 private:
  redis_store(const redis_store&);
  void operator=(const redis_store&);
 public:
  redis_store() {};
  virtual ~redis_store() {};

 private:
  redisContext* context;
  redisReply* reply;
  std::string type_rank_;
  boost::mutex put_mutex_;
  boost::mutex get_mutex_;
  int port;
  struct timeval timeout;

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
};

template<class Object>
void redis_store<Object>::init(const std::string& object_type, const int rank) {
  DLOG_ASSERT(!FLAGS_store_ip.empty())
      << "FLAG failure. You have to set IP address from commandline. "
      << " Run with --help, "
      << " and see how to run Differential Execution with file systems.";
  port = 6379;
  timeout = { 1, 500000 };  /* 1.5 seconds */
  context = redisConnectWithTimeout(FLAGS_store_ip.c_str(), port, timeout);

  if (context == NULL) {
    std::cerr << "Connection error. Fail to allocate redis context \n"
              << " Check redis server connection and IP address." << std::endl;
    exit(1);
  }
  if (context->err) {
    std::cerr << "Connection error. " << context->errstr << std::endl;
    redisFree(context);
    exit(1);
  }

  std::stringstream ss;
  ss << rank;
  type_rank_ = object_type + ss.str();
};

template<class Object>
void redis_store<Object>::init_read(const std::string& object_type,
                                    const int rank) {
  init(object_type, rank);
};

template<class Object>
void redis_store<Object>::finish() {
};

template<class Object>
void redis_store<Object>::clear_db() {
};

template<class Object>
void redis_store<Object>::put(const timestamp& key,
                              const long lp_id,
                              const Object& value) {
  boost::lock_guard<boost::mutex> guard(put_mutex_);
  /* serialize value */
  std::stringstream ss;
  boost::archive::text_oarchive ar(ss);
  ar << value;
  std::string binary = ss.str();

  /* put event */
  std::stringstream lpid_ss_;
  lpid_ss_ << lp_id;
  std::stringstream time_ss_;
  time_ss_ << key.time();
  std::string command = "ZADD " + type_rank_ + lpid_ss_.str() + " " + time_ss_.str() + " %b";
  reply = (redisReply*) redisCommand(context, command.c_str(), binary.c_str(), binary.size());
  freeReplyObject(reply);
};

template<class Object>
void redis_store<Object>::put_range(const std::vector<timestamp>& keys,
                                    const std::vector<obj_ptr>& values,
                                    const long lp_id) {
  for (int i = 0; i < keys.size(); ++i) {
    put(keys[i], lp_id, *(values[i]));
  }
};

template<class Object>
void redis_store<Object>::get(const timestamp& key,
                              const long lp_id, obj_ptr& ret) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  std::stringstream lpid_ss_;
  lpid_ss_ << lp_id;
  std::stringstream time_ss_;
  time_ss_ << key.time();
  std::string command = "ZRANGEBYSCORE " + type_rank_ + lpid_ss_.str() + " " + time_ss_.str() + " " + time_ss_.str();
  reply = (redisReply*) redisCommand(context, command.c_str());
  if (reply->type == REDIS_REPLY_ARRAY) {
    Object obj;
    for (int i = 0; i < reply->elements; ++i) {
      std::stringstream ss;
      ss << reply->element[i]->str;
      boost::archive::text_iarchive iar(ss);
      iar >> obj;
      if (obj.id() == key.id()) break;
    }
    ret = boost::make_shared<Object>(obj);
  }
  freeReplyObject(reply);
};

template<class Object>
void redis_store<Object>::get_prev(const timestamp& key,
                                   const long lp_id,
                                   obj_ptr& ret,
                                   timestamp& time_of_ret) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  std::stringstream lpid_ss_;
  lpid_ss_ << lp_id;
  std::stringstream time_ss_;
  time_ss_ << key.time();
  for (long back_time = 1;; back_time *= 2) {
    std::stringstream prev_ss_;
    long prev_time_ = key.time() > back_time ? key.time() - back_time : 0;
    prev_ss_ << prev_time_;
    std::string command = "ZRANGEBYSCORE " + type_rank_ + lpid_ss_.str()
        + " " + prev_ss_.str() + " (" + time_ss_.str();
    if (prev_ss_.str() >= time_ss_.str()) {
      command = "ZRANGEBYSCORE " + type_rank_ + lpid_ss_.str()
          + " " + prev_ss_.str() + " " + time_ss_.str();
    }
    reply = (redisReply*) redisCommand(context, command.c_str());
    if (reply->type == REDIS_REPLY_ARRAY) {
      Object obj;
      if (reply->elements > 0) {
        std::stringstream ss;
        ss << reply->element[reply->elements - 1]->str;
        boost::archive::text_iarchive iar(ss);
        iar >> obj;
        ret = boost::make_shared<Object>(obj);
        std::string binary = reply->element[reply->elements - 1]->str;
        std::string zscore_command = "ZSCORE " + type_rank_ + lpid_ss_.str() + " %b";
        freeReplyObject(reply);
        reply = (redisReply*) redisCommand(context, zscore_command.c_str(), binary.c_str(), binary.size());
        long time_ = atol(reply->str);
        time_of_ret = timestamp(time_, ret->id());
        freeReplyObject(reply);
        return;
      }
    }
    freeReplyObject(reply);
  }
};

template<class Object>
void redis_store<Object>::get_range(const timestamp& from,
                                    const timestamp& to,
                                    const long lp_id,
                                    std::vector<obj_ptr >& ret_obj) {
  boost::lock_guard<boost::mutex> guard(get_mutex_);
  if (from > to) return;
  /* generate key */
  std::stringstream lpid_ss_;
  lpid_ss_ << lp_id;
  std::stringstream from_ss_;
  from_ss_ << from.time();
  std::stringstream to_ss_;
  to_ss_ << to.time();
  std::string command = "ZRANGEBYSCORE " + type_rank_ + lpid_ss_.str()
                          + " " + from_ss_.str() + " (" + to_ss_.str();
  reply = (redisReply*) redisCommand(context, command.c_str());
  if (reply->type == REDIS_REPLY_ARRAY) {
    for (int i = 0; i < reply->elements; ++i) {
      std::stringstream ss;
      ss << reply->element[i]->str;
      boost::archive::text_iarchive iar(ss);
      Object obj;
      iar >> obj;
      if (obj.receive_time() == from.time() && obj.id() < from.id()) {
        continue;
      }
      ret_obj.push_back(boost::make_shared<Object>(obj));
    }
  }
  freeReplyObject(reply);
};

} /* namespace scalesim */

#endif /* SCALESIM_LOGICAL_PROCESS_STORE_REDIS_STORE_HPP_ */
