/*
 * global_sync.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_COM_MPI_GLOBAL_SYNC_HPP_
#define SCALESIM_COM_MPI_GLOBAL_SYNC_HPP_

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <glog/logging.h>

namespace scalesim {

template<class App>
class mpi_gsync {
 private:
  mpi_gsync(const mpi_gsync&);
  void operator=(const mpi_gsync&);

  mpi_gsync(): transit_counter_(0),
               is_red_(false),
               processing_ev_interval_(0),
               local_min_(timestamp::null()),
               gvt_(timestamp::zero()),
               gvt_counter_(counter::instance("GVT")) {};

 private:
  int transit_counter_;
  bool is_red_;
  int processing_ev_interval_;
  timestamp local_min_; boost::mutex mutex_;
  timestamp gvt_;
  counter* gvt_counter_;
  const boost::mpi::communicator* comm_world_;

 public:
  virtual ~mpi_gsync() {};
  static mpi_gsync* instance() {
    static mpi_gsync* instance_;
    if (!instance_) { instance_ = new mpi_gsync; }
    return instance_;
  };

  static void del_instance() {
    mpi_gsync* instance_ = mpi_gsync::instance();
    if (instance_) { delete instance_; instance_ = 0; }
  };

  void init(const boost::mpi::communicator* comm_world) {
    comm_world_ = comm_world;
  };
  void update_local(const timestamp& local_min_);
  void increment_interval();
  timestamp get_gvt();
  void check_sync();
  void increment_transit_conut() { ++transit_counter_; };
  void decrement_transit_conut() { --transit_counter_; };
  void reset_counter() { transit_counter_ = 0; };

  bool is_red() const { return is_red_; };
 private:
  int reduce_white_transit_num();
};

template<class App>
void mpi_gsync<App>::update_local(const timestamp& local_min) {
  boost::lock_guard<boost::mutex> guard(mutex_);
  if (local_min_ == timestamp::null()) {
    local_min_ = local_min;
  } else {
    local_min_ = std::min(local_min_, local_min);
  }
};

template<class App>
void mpi_gsync<App>::increment_interval() {
  ++processing_ev_interval_;
};

template<class App>
timestamp mpi_gsync<App>::get_gvt() {
  return gvt_;
};

template<class App>
void mpi_gsync<App>::check_sync() {
  if (gvt_.time() >= App::finish_time()) { return; }

  /* local_min_ has not updated after last gvt computing */
  if (local_min_ == timestamp::null()) { return; }

  /*
   * Interval between second cut and next first cut
   * The global_interval is a interval size between global virtual time.
   * Too small value causes wrong results because of events in send buffer.
   */
  if (processing_ev_interval_ < App::global_cut_interval()) { return; }

  /* make first cut (change RED from WHITE) */
  if (!is_red_) {
    is_red_ = true;
    return;
  }

  /* make second cut */
  if (is_red_) {
    boost::lock_guard<boost::mutex> guard(mutex_);
    stopwatch::instance("GlobalSync")->start();

    /* checking WHITE transit messages for making second cut */
    if (reduce_white_transit_num() <= 0) {
      /* make second cut */
      is_red_ = false;
      processing_ev_interval_ = 0;

      /* compute GVT */
      timestamp new_gvt_
          = boost::mpi::all_reduce(*comm_world_, local_min_,
              boost::mpi::minimum<timestamp>());

      DLOG_ASSERT(new_gvt_.time() >= gvt_.time())
          << "new_global_virtual_time_: " << new_gvt_.time()
          << " global_virtual_time_: " << gvt_.time()
          << " Global virtual time fail. Check GSYNC_INTERVAL. "
             "Too small value causes wrong gvt because of send buffer.";

      gvt_ = new_gvt_;
      ++(*gvt_counter_);

      /* reset local_min_ */
      local_min_ = timestamp::null();
    }
    stopwatch::instance("GlobalSync")->stop();
  }
};

template<class App>
int mpi_gsync<App>::reduce_white_transit_num() {
  int ret = transit_counter_;
  ret = boost::mpi::all_reduce(*comm_world_, ret, std::plus<int>());
  return ret;
}

} /* namespace scalesim */

#endif /* SCALESIM_COM_MPI_GLOBAL_SYNC_HPP_ */
