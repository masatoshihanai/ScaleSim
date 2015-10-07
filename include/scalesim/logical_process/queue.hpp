/*
 * queue.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_LOGICAL_PROCESS_QUEUE_HPP_
#define SCALESIM_LOGICAL_PROCESS_QUEUE_HPP_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/unordered_map.hpp>
#include "glog/logging.h"
#include "scalesim/logical_process/store/store_base.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template <class App>
class eventq {
 private:
  eventq(const eventq&);
  void operator=(const eventq&);
  eventq(){};

 public:
  explicit eventq(long id):
      id_(id),
      released_time_(timestamp::zero()), stored_ev_time_(timestamp::zero()),
      min_loaded_ev_time_(timestamp::max()),
      released_cn_time_(timestamp::zero()), stored_cn_time_(timestamp::zero()),
      min_loaded_can_time_(timestamp::max()),
      outputted_time_(timestamp::zero()),
      std_out_count_(counter::instance("OutputtedEvent")) {};
  virtual ~eventq(){ };

 private:
  const long id_;
  std::map<timestamp, ev_ptr<App> > map_, can_map_;
  std::vector<std::pair<timestamp, ev_ptr<App> > > buffer_;
  timestamp released_time_, stored_ev_time_, min_loaded_ev_time_;
  timestamp released_cn_time_, stored_cn_time_, min_loaded_can_time_;
  timestamp outputted_time_;
  counter* std_out_count_;

 public:
  void buffering(const ev_ptr<App>& ev);
  timestamp merge_buffer(ev_vec<App>& new_cancels);
  void increment(ev_ptr<App>& new_ev, timestamp* local_time_);
  void pop_back_cancels(const timestamp& lbound, ev_vec<App>& new_cancels);
  void set_cancel(const ev_ptr<App>& ev);
  int size_ev_queue() { return map_.size(); };
  int size_can_queue() { return can_map_.size(); };
  void release(const timestamp& to);
  void release_cancel(const timestamp& to);
  void std_out(const timestamp& to);

  /* for initial exact-diff simulation */
  void store_event(const timestamp& to);
  void store_cancel(const timestamp& to);

  /* for repeating exact-diff simulation */
  void load_events(ev_vec<App>& events, const timestamp& from);
  void load_cancels(ev_vec<App>& cancels, const timestamp& from);
  void delete_ev(const timestamp& tmstmp);
  void delete_can(const timestamp& tmstmp);
}; /* class event_q */

template<class App>
void eventq<App>::buffering(const ev_ptr<App>& ev) {
  timestamp stmp(ev->receive_time(), ev->id());
  buffer_.push_back(std::pair<timestamp, ev_ptr<App> >(stmp, ev));
};

template<class App>
timestamp eventq<App>::merge_buffer(ev_vec<App>& new_cancels) {
  timestamp min_stmp = timestamp::max();
  /* insert events to queue from buffer */
  for (auto ev_it = buffer_.begin(); ev_it != buffer_.end(); ++ev_it) {
    if (ev_it->second->is_cancel()) {
      if (map_.erase(ev_it->first)) {
        min_stmp = std::min(min_stmp, ev_it->first);
      }
    } else { /* ev_it is normal event */
      map_.insert(*ev_it);
      min_stmp = std::min(min_stmp, ev_it->first);
    }
  }
  buffer_.clear();

  /* generate new cancels */
  auto can_it = can_map_.lower_bound(min_stmp);
  for (; can_it != can_map_.end(); ++can_it) {
    ev_ptr<App> new_cancel = boost::make_shared<event<App> >(*can_it->second);
    new_cancel->change_cancel();
    new_cancels.push_back(new_cancel);
  }
  can_map_.erase(can_it, can_map_.end());

  return min_stmp;
}; /* merge_buffer() */

template<class App>
void eventq<App>::increment(ev_ptr<App>& new_ev, timestamp* local_time_) {
  if (*local_time_ == timestamp::max()) return;

  /* get event */
  auto it = map_.lower_bound(*local_time_);
  if (it != map_.end()) {
    new_ev = it->second;
  } else { /* it == map.end() */
    local_time_->update(timestamp::max());
    return;
  }

  /* increment local minimal */
  if (++it != map_.end()) {
    local_time_->update(it->first);
  } else { /* it == map.end() */
    local_time_->update(timestamp::max());
  }
};

template<class App>
void eventq<App>::pop_back_cancels(const timestamp& lbound,
                                   ev_vec<App>& new_cancels) {
  /* get cancels higher than lower_bound */
  for (auto it = can_map_.lower_bound(lbound); it != can_map_.end(); ++it) {
    ev_ptr<App> new_cancel = boost::make_shared<App>(*it->second);
    new_cancel->change_cancel();
    new_cancels.push_back(new_cancel);
  }

  /* delete the gotten cancels */
  can_map_.erase(can_map_.lower_bound(lbound), can_map_.end());
};

template<class App>
void eventq<App>::set_cancel(const ev_ptr<App>& ev) {
  ev_ptr<App> cancel = boost::make_shared<event<App> >(*ev);
  cancel->change_cancel();
  timestamp key(cancel->send_time(), cancel->id());
  can_map_.insert(std::pair<timestamp, ev_ptr<App> >(key, cancel));
};

template<class App>
void eventq<App>::release(const timestamp& to) {
  /* release events */
  DLOG_ASSERT(released_time_ < to || released_time_ == to)
      << "Global time: " << to.time()
      << " is lower than released time: " << released_time_.time();
  map_.erase(map_.lower_bound(released_time_), map_.lower_bound(to));
  released_time_ = to;
};

template<class App>
void eventq<App>::release_cancel(const timestamp& to) {
  /* release cancels */
  DLOG_ASSERT(released_time_ < to || released_time_ == to)
      << "Global time: " << to.time()
      << " is lower than released time: " << released_cn_time_.time();
  can_map_.erase(can_map_.lower_bound(released_cn_time_), can_map_.lower_bound(to));
  released_cn_time_ = to;
};

template<class App>
void eventq<App>::store_event(const timestamp& to) {
  /* store events in Exact-Differential Simulation */
  for (auto it = map_.lower_bound(stored_ev_time_);
       it != map_.lower_bound(to); ++it) {
    timestamp key = it->first;
    ev_ptr<App> value = it->second;
    store<App>::ev_store()->put(key, id_, *value);
  }
  stored_ev_time_ = to;
};

template<class App>
void eventq<App>::store_cancel(const timestamp& to) {
  /* store cancels in Exact-Differential Simulation */
  for (auto it = can_map_.lower_bound(stored_cn_time_);
       it != can_map_.lower_bound(to); ++it) {
    timestamp key = it->first;
    ev_ptr<App> value = it->second;
    store<App>::can_store()->put(key, id_, *value);
  }
  stored_cn_time_ = to;
};

template <class App>
void eventq<App>::std_out(const timestamp& to) {
  for (auto it = map_.lower_bound(outputted_time_); it != map_.lower_bound(to);
       ++it) {
    it->second->result_out();
    ++(*std_out_count_);
  }
  outputted_time_ = to;
};

template<class App>
void eventq<App>::load_events(ev_vec<App>& events, const timestamp& from) {
  scalesim::store<App>::ev_store()
      ->get_range(from, min_loaded_ev_time_, id_, events);
  min_loaded_ev_time_ = from;
};

template<class App>
void eventq<App>::load_cancels(ev_vec<App>& cancels, const timestamp& from) {
  scalesim::store<App>::can_store()
      ->get_range(from, min_loaded_can_time_, id_, cancels);
  min_loaded_can_time_ = from;
};

template<class App>
void eventq<App>::delete_ev(const timestamp& tmstmp) {
  /* insert cancel event to buffer */
  ev_ptr<App> cancel
    = boost::make_shared<event<App> >(event<App>());
  cancel->change_cancel();
  std::pair<timestamp, ev_ptr<App> > pair_(tmstmp, cancel);
  buffer_.push_back(pair_);
};

template<class App>
void eventq<App>::delete_can(const timestamp& tmstmp) {
  auto it = can_map_.find(tmstmp);
  can_map_.erase(it);
};

template <class App>
class stateq {
 private:
  stateq();
  stateq(const stateq&);
  void operator=(const stateq&);
 public:
  explicit stateq(long id): id_(id),
      released_st_time(timestamp::zero()),
      stored_st_time(timestamp::zero()),
      clear_state_counter_(counter::instance("StateClear")){};
  virtual ~stateq(){};
 private:
  const long id_;
  std::map<timestamp, st_ptr<App> > state_map;
  timestamp released_st_time;
  timestamp stored_st_time;
  counter* clear_state_counter_;
 public:
  void get_state(st_ptr<App>& new_state);
  void push_back_state(const st_ptr<App>& state, const timestamp& time);
  void rollback_state(const timestamp& lower_bound);
  void release(const timestamp& to);

  /* for exact-diff simulation */
  void store_state(const timestamp& to);
  void load_prev_st(st_ptr<App>& new_state, timestamp& time_of_state,
                    const timestamp& time);
}; /* class stateq */

template<class App>
void stateq<App>::get_state(st_ptr<App>& new_state) {
  auto it = state_map.end();
  --it;
  new_state = it->second;
};

template<class App>
void stateq<App>::push_back_state(const st_ptr<App>& state,
                                  const timestamp& time) {
  state_map.insert(std::pair<timestamp, st_ptr<App> >(time, state));
};

template<class App>
void stateq<App>::rollback_state(const timestamp& lower_bound) {
  auto it = state_map.lower_bound(lower_bound);
  state_map.erase(it, state_map.end());
};

template<class App>
void stateq<App>::release(const timestamp& to) {
  DLOG_ASSERT(released_st_time < to || released_st_time == to)
      << " Global time: " << to.time()
      << " is lower than released time: " << released_st_time.time();
  auto from_it = state_map.lower_bound(released_st_time);
  auto to_it = state_map.lower_bound(to);
  *clear_state_counter_ += (std::distance(from_it, to_it));
  state_map.erase(from_it, to_it);
  released_st_time = to;
};

template<class App>
void stateq<App>::store_state(const timestamp& to) {
  DLOG_ASSERT(released_st_time < to || released_st_time == to)
      << "Global time: " << to.time()
      << " is lower than stored time: " << stored_st_time.time();
  for (auto it = state_map.lower_bound(stored_st_time);
       it != state_map.lower_bound(to); ++it) {
    timestamp key = it->first;
    st_ptr<App> value = it->second;
    store<App>::st_store()->put(key, id_, *value);
  }
  stored_st_time = to;
};

template<class App>
void stateq<App>::load_prev_st(st_ptr<App>& new_state,
                               timestamp& time_of_state,
                               const timestamp& time) {
  if (time == timestamp::max()) { return; }
  store<App>::st_store()->get_prev(time, id_, new_state, time_of_state);
};

} /* namespace scalesim */

#endif /* SCALESIM_LOGICAL_PROCESS_QUEUE_HPP_ */
