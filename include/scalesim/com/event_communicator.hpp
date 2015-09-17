/*
 * event_communicator.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_COM_EVENT_COMMUNICATOR_HPP_
#define SCALESIM_COM_EVENT_COMMUNICATOR_HPP_

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include "scalesim/com/mpi/mpi_runner.hpp"
#include "scalesim/logical_process.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template <class App>
class event_com {
 private:
  event_com(const event_com&);
  void operator=(const event_com&);
  event_com(){};
 public:
  virtual ~event_com(){};
 private:
  parti_ptr partition_;
  lp_mngr<App>* lp_manager_;
  boost::mutex send_lock;
  int rank_; int rank_size_;
 public:
  static event_com* instance() {
    static event_com* instance_;
    if (!instance_) { instance_ = new event_com; }
    return instance_;
  };
  static void del_instance() {
    event_com* instance_ = event_com::instance();
    if (instance_) { delete instance_; instance_ = NULL; }
  };

  void init(lp_mngr<App>* lp_manager);
  void start();
  void stop();
  void finish();

  void send_event(const ev_ptr<App>& event);
  void receive_event(const ev_ptr<App>& event);

  void barrier(bool& wait);

  template<class Obj>
  void shuffle(bool& wait,
               std::vector<boost::shared_ptr<const Obj> >& ret,
               std::vector<boost::shared_ptr<const Obj> >& events,
               const parti_ptr parti);

  void reduce_sum(bool& wait, long& ret, const long val);

  int rank() const ;
  int rank_size() const;
  void reset();
};

template<class App>
void event_com<App>::init(lp_mngr<App>* lp_manager) {
  lp_manager_ = lp_manager;
  boost::function<void(const ev_ptr<App>&)> call_back_f
     = boost::bind(&event_com<App>::receive_event, this, _1);
  mpi_thr<App>::instance()->init(call_back_f);
  rank_ = mpi_thr<App>::instance()->mpi_rank();
  rank_size_ = mpi_thr<App>::instance()->mpi_size();
  bool wait_barrier_ = true;
  mpi_thr<App>::instance()->barrier(wait_barrier_);
  while(wait_barrier_);
};

template<class App>
void event_com<App>::start() {
  mpi_thr<App>::instance()->start();
}

template<class App>
void event_com<App>::stop() {
  mpi_thr<App>::instance()->stop();
}

template<class App>
void event_com<App>::finish() {
  mpi_thr<App>::instance()->finish();
  mpi_thr<App>::del_instance();
};

template<class App>
void event_com<App>::barrier(bool& wait_barrier)
  { mpi_thr<App>::instance()->barrier(wait_barrier); };

template<class App>
void event_com<App>::send_event(const ev_ptr<App>& event) {
  long destination_rank = (*lp_manager_->partition())[event->destination()];
  if (mpi_thr<App>::instance() == NULL ||
      destination_rank == mpi_thr<App>::instance()->mpi_rank()) {
    /* inner communication */
    lp<App>* lp_;
    lp_manager_->get_lp(lp_, event->destination());
    lp_->buffer(event);
  } else {
    /* rank to rank communication */
    mpi_thr<App>::instance()->buffer_send_message(destination_rank, event);
  }
};

template<class App>
void event_com<App>::receive_event(const ev_ptr<App>& event) {
  lp<App>* lp_;
  lp_manager_->get_lp(lp_, event->destination());
  lp_->buffer(event);
};

template<class App>
template<class Obj>
void event_com<App>::shuffle(bool& wait,
                             std::vector<boost::shared_ptr<const Obj> >& ret,
                             std::vector<boost::shared_ptr<const Obj> >& objs,
                             parti_ptr parti) {
  mpi_thr<App>::instance()->template shuffle<Obj>(wait, ret, objs, parti);
};

template<class App>
void event_com<App>::reduce_sum(bool& wait, long& ret, long val)
  { mpi_thr<App>::instance()->reduce_sum(wait, ret, val); };

template<class App>
int event_com<App>::rank() const { return rank_; };

template<class App>
int event_com<App>::rank_size() const { return rank_size_; };

template<class App>
void event_com<App>::reset() { mpi_thr<App>::instance()->reset(); };

} /* namespae scalesim */

#endif /* SCALESIM_COM_EVENT_COMMUNICATOR_HPP_ */
