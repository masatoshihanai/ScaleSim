/*
 * process_scheduler.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#ifndef SCALESIM_LOGICAL_PROCESS_PROCESS_SCHEDULER_HPP_
#define SCALESIM_LOGICAL_PROCESS_PROCESS_SCHEDULER_HPP_

#include <map>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include "scalesim/util.hpp"

namespace scalesim {

/*
 * This is a event process scheduler based on low time stamp first (ltsf) queue.
 * Ltsf queue was proposed in WARPED in Univ. of Cincinnati.
 *   http://secs.ceas.uc.edu/~paw/research/warped/
 */
template<class App>
class scheduler {
 private:
  scheduler(const scheduler&);
  void operator=(const scheduler&);
 public:
  scheduler(){};
  virtual ~scheduler(){};

 private:
  std::map<timestamp, long> ltsf_queue_;        /* <timestamp, lp id> */
  boost::unordered_map<long, timestamp> index_; /* <lp id, timestamp> */
  boost::unordered_set<long> active_lp_;
  boost::mutex mutex_;

 public:
  static int local_parti(long lp_id, int num_scheduler);
  long dequeue();
  void queue(const timestamp& tmstmp, long lp_id);
  timestamp min_locals();
  boost::unordered_set<long>* active_lp();
  void reset_active_lp();
};

template<class App>
int scheduler<App>::local_parti(long lp_id, int num_scheduler) {
  return lp_id % num_scheduler;
};

template<class App>
long scheduler<App>::dequeue() {
  boost::lock_guard<boost::mutex> gurad(mutex_);
  auto it = ltsf_queue_.begin();
  if (it == ltsf_queue_.end() || it->first == timestamp::max()) {
    return -1;
  }
  long ret = it->second;
  index_.erase(ret);
  ltsf_queue_.erase(it);
  active_lp_.insert(ret);
  return ret;
};

template<class App>
void scheduler<App>::queue(const timestamp& tmstmp, long lp_id) {
  boost::lock_guard<boost::mutex> gurad(mutex_);
  if (index_.count(lp_id) > 0) {
    timestamp old_time = index_[lp_id];
    if (old_time < tmstmp) { return; }
    ltsf_queue_.erase(old_time);
    index_.erase(lp_id);
  }

  ltsf_queue_.insert(std::pair<timestamp, long>(tmstmp, lp_id));
  index_.insert(std::pair<long, timestamp>(lp_id, tmstmp));
};

template<class App>
timestamp scheduler<App>::min_locals() {
  boost::lock_guard<boost::mutex> gurad(mutex_);
  if (ltsf_queue_.begin() == ltsf_queue_.end()) {
    return timestamp::max();
  }
  return ltsf_queue_.begin()->first;
};

template<class App>
boost::unordered_set<long>* scheduler<App>::active_lp()
  { return &active_lp_; };

template<class App>
void scheduler<App>::reset_active_lp()
  { active_lp_.clear(); };

} /* namespace scalesim */

#endif /* SCALESIM_LOGICAL_PROCESS_PROCESS_SCHEDULER_HPP_ */
