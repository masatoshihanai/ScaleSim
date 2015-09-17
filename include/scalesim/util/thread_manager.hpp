/*
 * thread_manager.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_UTIL_THREAD_MANAGER_HPP_
#define SCALESIM_UTIL_THREAD_MANAGER_HPP_

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace scalesim {

class thr_pool {
 private:
  boost::asio::io_service io_service_;
  boost::shared_ptr<boost::asio::io_service::work> work_;
  boost::thread_group thread_group_;
  int num_active_thr; boost::mutex mutex_;
 private:
  thr_pool();
  thr_pool(const thr_pool&);
  void operator=(const thr_pool&);
 public:
  explicit thr_pool(const int pool_size): num_active_thr(0) {
    work_.reset(new boost::asio::io_service::work(io_service_));
    for (int i = 0; i < pool_size; ++i) {
      thread_group_.create_thread(
          boost::bind(static_cast<size_t (boost::asio::io_service::*)()>
              (&boost::asio::io_service::run), &io_service_));
    }
  };
  virtual ~thr_pool() {
    work_.reset();
    thread_group_.join_all();
    io_service_.stop();
  };
 public:
  template <class Func>
  void post(Func f) {
    boost::lock_guard<boost::mutex> guard(mutex_);
    ++num_active_thr;
    io_service_.post(f);
  };
  void callback_end() {
    boost::lock_guard<boost::mutex> guard(mutex_);
    --num_active_thr;
  };
  void wait_all() {
    while (num_active_thr != 0);
  };
};

} /* namespace scalesim */

#endif /* SCALESIM_UTIL_THREAD_MANAGER_HPP_ */
