/*
 * mpi_runner.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_COM_MPI_MPI_RUNNER_HPP_
#define SCALESIM_COM_MPI_MPI_RUNNER_HPP_

#include <vector>
#include <iostream>
#include <boost/function.hpp>
#include <boost/mpi.hpp>
#include <boost/mpi/collectives.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <glog/logging.h>

#include "scalesim/com/mpi/collection.hpp"
#include "scalesim/com/mpi/global_sync.hpp"
#include "scalesim/com/mpi/sender_receiver.hpp"
#include "scalesim/simulation/sim_obj.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template<class App>
class mpi_thr {
  typedef boost::shared_ptr<mpi_thr<App> > mpi_thr_ptr;
  typedef std::vector<boost::mpi::request> requests;
 private:
  mpi_thr(): init_(false), loop_active_(false), finish_(false) {};
  void operator=(const mpi_thr&);

 private:
  bool init_; bool loop_active_; bool finish_;
  mpi_collection<App> collection_;
  mpi_sender<App> sender_;
  mpi_receiver<App> receiver_;
  boost::thread* thread_;
  boost::function<void(ev_ptr<App>)> call_back_function_;
  boost::mpi::environment env_;
  boost::mpi::communicator comm_world_;

 public:
  virtual ~mpi_thr() {};
  static mpi_thr<App>* instance() {
    static mpi_thr<App>* instance_;
    if (!instance_) {
      instance_ = new mpi_thr<App>();
    }
    return instance_;
  };
  static void del_instance() {
    mpi_thr<App>* instance_ = mpi_thr<App>::instance();
    if (instance_ != NULL) {
      delete instance_;
      instance_ = NULL;
    }
  };

  void init(const boost::function<void(const ev_ptr<App>&)>& callback_f);
  void start() { loop_active_ = true; init_ = false; };
  void stop() { finish_ = true; loop_active_ = false; };
  void finish();
  void reset();

  int mpi_rank() { return comm_world_.rank(); };
  int mpi_size() { return comm_world_.size(); };

  void buffer_send_message(int rank, const ev_ptr<App>& event);

  void barrier(bool& wait_barrier);

  template<class Obj>
  void shuffle(bool& wait,
               std::vector<boost::shared_ptr<const Obj> >& ret,
               std::vector<boost::shared_ptr<const Obj> >& objs,
               const parti_ptr& parti);

  void reduce_sum(bool& wait, long& ret, long val);

 private:
  void loop();

 private:
  template<class Obj, class T = void>
  class shuffle_func {
    friend class mpi_thr;
    static void shuffle_impl(bool& wait,
                             std::vector<boost::shared_ptr<const Obj> >& ret,
                             std::vector<boost::shared_ptr<const Obj> >& objs,
                             const parti_ptr& parti);
  };

  template<class T>
  class shuffle_func<event<App>, T> {
    friend class mpi_thr;
    static void shuffle_impl(bool& wait,
                             ev_vec<App>& ret,
                             ev_vec<App>& objs,
                             const parti_ptr& parti) {
      mpi_thr<App>::instance()
          ->collection_.wait_shuffle_event(wait, ret, objs, parti);
    };
  };

  template<class T>
  class shuffle_func<what_if<App>, T> {
    friend class mpi_thr;
    static void shuffle_impl(bool& wait,
                     std::vector<boost::shared_ptr<const what_if<App> > >& ret,
                     std::vector<boost::shared_ptr<const what_if<App> > >& objs,
                     const parti_ptr& parti) {
    mpi_thr<App>::instance()
        ->collection_.wait_shuffle_what_if(wait, ret, objs, parti);
    };
  };
};

template<class App>
void mpi_thr<App>::init(
    const boost::function<void(const ev_ptr<App>&)>& callback_f) {
  init_ = true;
  collection_.init(&comm_world_);
  sender_.init(&comm_world_);
  receiver_.init(&comm_world_);
  mpi_gsync<App>::instance()->init(&comm_world_);
  call_back_function_ = callback_f;
  thread_ = new boost::thread(boost::bind(&mpi_thr<App>::loop, this));
}

template<class App>
void mpi_thr<App>::finish() {
  finish_ = false;
  thread_->join();
  delete thread_;
};

template<class App>
void mpi_thr<App>::reset() {
  mpi_gsync<App>::instance()->reset_counter();
  sender_.reset();
};

template<class App>
void mpi_thr<App>::buffer_send_message(int rank, const ev_ptr<App>& event) {
  sender_.buffer_sendevent(rank, event);
};

template<class App>
void mpi_thr<App>::barrier(bool& wait_barrier) {
  DLOG_ASSERT(!loop_active_)
      << "Collection cannot be invoked during loop is active. "
      << "It can be invoked only after start() or after stop()";
  collection_.wait_barrier(wait_barrier);
};

template<class App>
template<class Obj>
void mpi_thr<App>::shuffle(bool& wait,
                           std::vector<boost::shared_ptr<const Obj> >& ret,
                           std::vector<boost::shared_ptr<const Obj> >& objs,
                           const parti_ptr& parti) {
  shuffle_func<Obj>::shuffle_impl(wait, ret, objs, parti);
};

template<class App>
void mpi_thr<App>::reduce_sum(bool& wait, long& ret, long val) {
  DLOG_ASSERT(!loop_active_)
      << "Collection cannot be invoked during loop is active. "
      << "It can be invoked only after start() or after stop()";
  collection_.wait_reduce_sum(wait, ret, val);
};

template<class App>
void mpi_thr<App>::loop() {
  while (init_) {
    /* wait global communication */
    collection_.check_collection();
  }

  while (loop_active_) {
    /* global sync */
    mpi_gsync<App>::instance()->check_sync();

    /* send and receive */
    sender_.async_send();
    receiver_.receive(call_back_function_);
    sender_.check_send();
  }

  while (finish_) {
    /* wait global communication */
    collection_.check_collection();
  }
};

} /* namespace scalesim */

#endif /* SCALESIM_COM_MPI_MPI_RUNNER_HPP_ */
