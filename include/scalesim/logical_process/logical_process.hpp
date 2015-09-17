/*
 * logical_process.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_LOGICAL_PROCESS_LOGICAL_PROCESS_HPP_
#define SCALESIM_LOGICAL_PROCESS_LOGICAL_PROCESS_HPP_

#include <algorithm>
#include <string>
#include <vector>
#include <cassert>
#include <boost/shared_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/unordered_map.hpp>
#include <gflags/gflags.h>
#include "scalesim/logical_process/queue.hpp"
#include "scalesim/logical_process/store/store_base.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template <class App>
class lp {
 private:
  lp(const lp&);
  void operator=(const lp&);
 public:
  lp():id_(0),
       local_time_(timestamp::max()),
       buf_min_time_(timestamp::max()),
       eventq_(new eventq<App>(0)),
       stateq_(new stateq<App>(0)),
       event_counter_(counter::instance("Events")),
       cancel_counter_(counter::instance("Cancels")){};
  explicit lp(long id):
       id_(id),
       local_time_(timestamp::max()),
       buf_min_time_(timestamp::max()),
       eventq_(new eventq<App>(id)),
       stateq_(new stateq<App>(id)),
       event_counter_(counter::instance("Events")),
       cancel_counter_(counter::instance("Cancels")){};
  virtual ~lp() {
    if (eventq_) { delete eventq_; }
    if (stateq_) { delete stateq_; }
  };
 private:
  const long id_;
  timestamp local_time_;
  timestamp buf_min_time_;
  eventq<App>* eventq_;
  stateq<App>* stateq_;
  boost::mutex mutex_;
  counter* event_counter_;
  counter* cancel_counter_;

 public:
  void init_state(const st_ptr<App>& state);
  void init_event(const ev_ptr<App>& initial_event);
  timestamp local_time() const;
  long id() const;
  void buffer(const ev_ptr<App>& event);
  void flush_buf(std::vector<ev_ptr<App> >& new_cancels);
  void dequeue_event(ev_ptr<App>& new_ev);
  void get_state(st_ptr<App>& new_state);
  void set_cancel(const ev_ptr<App>& original_event);
  void update_state(const st_ptr<App>& state, const timestamp& time);
  void clear_old_ev(const timestamp& to);
  void clear_old_st(const timestamp& to);
  void std_out(const timestamp& from, const timestamp& to);

  /* for repeating what-if simulation */
  void init_state(const st_ptr<App>& state, const timestamp& time);
  void load_events(ev_vec<App>& events, const timestamp& from);
  void load_cancels(ev_vec<App>& cancels, const timestamp& from);
  void load_prev_state(st_ptr<App>& new_state, timestamp& time_of_state,
                       const timestamp& time);
  void delete_ev(const timestamp& time);
  void delete_can(const timestamp& time);
};

template<class App>
using logical_processes = boost::unordered_map<long, lp<App>*>;

template<class App>
void lp<App>::init_state(const st_ptr<App>& state)
  { stateq_->push_back_state(state, timestamp::null()); };

template<class App>
void lp<App>::init_event(const ev_ptr<App>& initial_event)
  { buffer(initial_event); };

template<class App>
timestamp lp<App>::local_time() const
  { return std::min(local_time_, buf_min_time_); };

template<class App>
long lp<App>::id() const { return id_; };

/* thread safe */
template<class App>
void lp<App>::buffer(const ev_ptr<App>& event) {
  boost::lock_guard<boost::mutex> guard(mutex_);
  eventq_->buffering(event);
  buf_min_time_
      = std::min(buf_min_time_, timestamp(event->receive_time(), event->id()));
};

template<class App>
void lp<App>::flush_buf(std::vector<ev_ptr<App>>& new_cancels) {
  boost::lock_guard<boost::mutex> guard(mutex_);
  if (FLAGS_diff_repeat) {
    const timestamp from = std::min(buf_min_time_, local_time_);
    /* load event and cancel */
    ev_vec<App> load_events_;
    load_events(load_events_, from);
    for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
      eventq_->buffering(*it);
    }
    ev_vec<App> load_cancels_;
    load_cancels(load_cancels_, from);
    for (auto it = load_cancels_.begin(); it != load_cancels_.end(); ++it) {
      set_cancel(*it);
    }

    /* load state */
    st_ptr<App> load_state_;
    timestamp load_tmstmp_;
    load_prev_state(load_state_, load_tmstmp_, from);
    if (load_state_) { init_state(load_state_, load_tmstmp_); }
  }

  timestamp lower_bound_ = eventq_->merge_buffer(new_cancels);
  stateq_->rollback_state(lower_bound_);

  local_time_ = std::min(lower_bound_, local_time_);
  buf_min_time_ = timestamp::max();
  (*cancel_counter_) += new_cancels.size();
};

template<class App>
void lp<App>::dequeue_event(ev_ptr<App>& new_ev) {
  eventq_->increment(new_ev, &local_time_);
  if (new_ev) { ++(*event_counter_); }
};

template<class App>
void lp<App>::get_state(st_ptr<App>& new_state)
  { stateq_->get_state(new_state); };

template<class App>
void lp<App>::set_cancel(const ev_ptr<App>& original_event)
  { eventq_->set_cancel(original_event); };

template<class App>
void lp<App>::update_state(const st_ptr<App>& state, const timestamp& time)
  { stateq_->push_back_state(state, time); };

template<class App>
void lp<App>::clear_old_ev(const timestamp& to) {
  if (FLAGS_diff_init) {
    eventq_->store_event(to);
    eventq_->store_cancel(to);
  }
  eventq_->release(to);
  eventq_->release_cancel(to);
};

template<class App>
void lp<App>::clear_old_st(const timestamp& to) {
  if (FLAGS_diff_init) {
    stateq_->store_state(to);
  }
  stateq_->release(to);
};

template<class App>
void lp<App>::std_out(const timestamp& from, const timestamp& to)
  { eventq_->std_out(from, to); };

template<class App>
void lp<App>::init_state(const st_ptr<App>& state, const timestamp& time)
  { stateq_->push_back_state(state, time); };

template<class App>
void lp<App>::load_events(ev_vec<App>& events, const timestamp& from)
  { eventq_->load_events(events, from); };

template<class App>
void lp<App>::load_cancels(ev_vec<App>& cancels, const timestamp& from)
  { eventq_->load_cancels(cancels, from); };

template<class App>
void lp<App>::load_prev_state(st_ptr<App>& new_state, timestamp& time_of_state,
                              const timestamp& time)
  { stateq_->load_prev_st(new_state, time_of_state, time); };

template<class App>
void lp<App>::delete_ev(const timestamp& tmstmp)
  { eventq_->delete_ev(tmstmp); };

template<class App>
void lp<App>::delete_can(const timestamp& tmstmp)
  { eventq_->delete_can(tmstmp); };

template <class App>
class lp_mngr {
 private:
  lp_mngr(const lp_mngr&);
  void operator=(const lp_mngr&);
  lp_mngr(){};
 public:
  virtual ~lp_mngr(){};
 private:
  parti_ptr partition_;
  parti_indx_ptr partition_index;
  logical_processes<App> lps_; /* <kay, value> = <lp_id, lp> */
 public:
  static lp_mngr<App>* instance() {
    static lp_mngr<App>* instance_;
    if (!instance_) { instance_ = new lp_mngr<App>; }
    return instance_;
  };
  static void del_instance() {
    lp_mngr<App>* instance_ = lp_mngr<App>::instance();
    if (instance_) { delete instance_; instance_ = 0; }
  };

  logical_processes<App>& get_lps () { return lps_; };

  void init_partition (const std::pair<parti_ptr, parti_indx_ptr>& parti);
  void init_lps(int this_rank, int rank_size);
  void delete_lps(int this_rank);
  void get_lp(lp<App>*& lp, long id);
  parti_ptr partition();
};

template<class App>
void lp_mngr<App>::init_partition(
    const std::pair<parti_ptr, parti_indx_ptr>& parti) {
  partition_ = parti.first;
  partition_index = parti.second;
};

template<class App>
void lp_mngr<App>::init_lps(int this_rank, int rank_size) {
  DLOG_ASSERT(partition_index->size() > 0)
      << "Init partition fails. Check partition file.";

  /* initiate logical processes */
  auto rng = partition_index->equal_range(this_rank);
  for (auto it = rng.first; it != rng.second; ++it) {
    lps_[it->second] = new lp<App>(it->second);
  }

  /* initiate stores */
  if (FLAGS_diff_init) {
    LOG_IF(INFO, this_rank == 0) << "Initiate stores";
    store<App>::init(this_rank);
  }

  if (FLAGS_diff_repeat) {
    LOG_IF(INFO, this_rank == 0) << "Initiate stores in read-only mode";
    store<App>::init_read(this_rank);
  }
};

template<class App>
void lp_mngr<App>::delete_lps(int this_rank) {
  /* delete logical processes */
  auto rng_ = partition_index->equal_range(this_rank);
  for (auto it = rng_.first; it != rng_.second; ++it) {
    delete lps_[it->second];
    lps_[it->second] = 0;
  }

  /* delete stores */
  if (FLAGS_diff_init) {
    store<App>::finish();
  }
  if (FLAGS_diff_repeat) {
    store<App>::finish();
  }
};

template<class App>
void lp_mngr<App>::get_lp(lp<App>*& lp, long id) {
  DLOG_ASSERT(lps_[id]) << "Fail to get logical process " << id;
  lp = lps_[id];
};

template<class App>
parti_ptr lp_mngr<App>::partition() {
  DLOG_ASSERT(partition_) << "Fail to get partition";
  return partition_;
};

} /* namespace scalesim */

#endif /* SCALESIM_TIMEWARP_LP_HPP_ */
