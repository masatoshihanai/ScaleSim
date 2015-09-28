/*
 * gvt_communicator.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_COM_GVT_COMMUNICATOR_HPP_
#define SCALESIM_COM_GVT_COMMUNICATOR_HPP_

namespace scalesim {

template <class App>
class gvt_com {
 private:
  gvt_com(const gvt_com&);
  void operator=(const gvt_com&);
 public:
  gvt_com(): global_time_(timestamp::zero()),
             global_time_prev(timestamp::zero()){};
  virtual ~gvt_com(){};
 private:
  timestamp global_time_;
  timestamp global_time_prev;
 public:
  void update_local_min(const timestamp& local_min);
  bool check_updated();
  void increment_gsync_interval();
  timestamp gtime();
  timestamp gtime_prev();
};

template<class App>
void gvt_com<App>::update_local_min(const timestamp& local_min) {
  mpi_gsync<App>::instance()->update_local(local_min);
};

template<class App>
bool gvt_com<App>::check_updated() {
  if (mpi_gsync<App>::instance()->get_gvt() > global_time_) {
    global_time_prev = global_time_;
    global_time_ = mpi_gsync<App>::instance()->get_gvt();
    return true;
  }
  return false;
};

template<class App>
void gvt_com<App>::increment_gsync_interval() {
  mpi_gsync<App>::instance()->increment_interval();
};

template<class App>
timestamp gvt_com<App>::gtime() {
  return global_time_;
};

template<class App>
timestamp gvt_com<App>::gtime_prev() {
  return global_time_prev;
};

} /* namespace scalesim */

#endif /* SCALESIM_COM_GVT_COMMUNICATOR_HPP_ */
