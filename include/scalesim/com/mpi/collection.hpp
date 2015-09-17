/*

 *
 *  Created on: Jul 29, 2015
 *      Author: masahanai
 */
/*
 * collection.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_COM_MPI_COLLECTION_HPP_
#define SCALESIM_COM_MPI_COLLECTION_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "scalesim/simulation/sim_obj.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template<class App>
class mpi_collection {
 private:
  mpi_collection(const mpi_collection&);
  void operator=(const mpi_collection&);
 public:
  mpi_collection(): wait_(false){};
  virtual ~mpi_collection(){};
 private:
  const boost::mpi::communicator* com_world_;
  bool wait_;
  bool* wait_barrier_;

  bool* wait_shuffle_ev_; ev_vec<App>* shuffle_ev_buf_;
  ev_vec<App>* shuffle_ev_ret_; parti_ptr shuffle_parti_;

  bool* wait_shuffle_what_if_;
  std::vector<boost::shared_ptr<const what_if<App> > >* shuffle_wi_buf_;
  std::vector<boost::shared_ptr<const what_if<App> > >* shuffle_wi_ret_;

  bool* wait_reduce_; long reduce_buf_; long* reduce_ret_;

 public:
  void init(const boost::mpi::communicator* comm_world) {
    com_world_ = comm_world;
  };

  void check_collection();

  void wait_barrier(bool& wait_barrier);

  void wait_shuffle_event(bool& wait, ev_vec<App>& ret,
                          ev_vec<App>& events, const parti_ptr& parti);

  void wait_shuffle_what_if(bool& wait_shuffle,
                     std::vector<boost::shared_ptr<const what_if<App> > >& ret,
                     std::vector<boost::shared_ptr<const what_if<App> > >& objs,
                     const parti_ptr& parti);

  void wait_reduce_sum(bool& wait_reduce, long& ret, const long val);

 private:
  void shuffle_event();
  void shuffle_what_if();
};

template<class App>
void mpi_collection<App>::check_collection() {
  if (wait_) {
    if (wait_barrier_ && *wait_barrier_) {
      com_world_->barrier();
      *wait_barrier_ = false;
      wait_barrier_ = NULL;
      wait_ = false;
    } else if (wait_shuffle_ev_ && *wait_shuffle_ev_) {
      shuffle_event();
      *wait_shuffle_ev_ = false;
      wait_shuffle_ev_ = NULL;
      wait_ = false;
    } else if (wait_shuffle_what_if_ && *wait_shuffle_what_if_) {
      shuffle_what_if();
      *wait_shuffle_what_if_ = false;
      wait_shuffle_what_if_ = NULL;
      wait_ = false;
    } else if (wait_reduce_ && *wait_reduce_) {
      *reduce_ret_
          = boost::mpi::all_reduce(*com_world_, reduce_buf_, std::plus<long>());
      *wait_reduce_ = false;
      wait_reduce_ = NULL;
      wait_ = false;
    }
  }
}

template<class App>
void mpi_collection<App>::wait_barrier(bool& wait_barrier) {
  wait_ = true;
  wait_barrier = true;
  wait_barrier_ = &wait_barrier;
}

template<class App>
void mpi_collection<App>::wait_shuffle_event(bool& wait_shuffle,
                                             ev_vec<App>& ret,
                                             ev_vec<App>& events,
                                             const parti_ptr& parti) {
  shuffle_ev_buf_ = &events;
  shuffle_ev_ret_ = &ret;
  shuffle_parti_ = parti;
  wait_ = true;
  wait_shuffle = true;
  wait_shuffle_ev_ = &wait_shuffle;
};

template<class App>
void mpi_collection<App>::wait_shuffle_what_if(bool& wait_shuffle,
                     std::vector<boost::shared_ptr<const what_if<App> > >& ret,
                     std::vector<boost::shared_ptr<const what_if<App> > >& objs,
                     const parti_ptr& parti) {
  shuffle_wi_buf_ = &objs;
  shuffle_wi_ret_ = &ret;
  shuffle_parti_ = parti;
  wait_ = true;
  wait_shuffle = true;
  wait_shuffle_what_if_ = &wait_shuffle;
}

template<class App>
void mpi_collection<App>::wait_reduce_sum(bool& wait_reduce,
                                          long& ret, const long val) {
  reduce_buf_ = val;
  reduce_ret_ = &ret;
  wait_ = true;
  wait_reduce = true;
  wait_reduce_ = &wait_reduce;
}

template<class App>
void mpi_collection<App>::shuffle_event() {
  /* calculate  maximum size of buffers */
  std::vector<int> msg_num_to_rank(com_world_->size(), 0);
  for (auto it = shuffle_ev_buf_->begin(); it != shuffle_ev_buf_->end(); ++it) {
    int target_rank = (*shuffle_parti_)[(*it)->destination()];
    ++msg_num_to_rank[target_rank];
  }
  int max_buf_size
      = *std::max_element(msg_num_to_rank.begin(), msg_num_to_rank.end());
  max_buf_size
      = boost::mpi::all_reduce(*com_world_, max_buf_size,
                               boost::mpi::maximum<int>());

  /* make events vectors, where events for rank i are in out_bufs[i] */
  std::vector<event<App> > out_bufs(com_world_->size()*max_buf_size,
                                    event<App>());
  std::vector<int> index(com_world_->size(), 0);
  for (auto it = shuffle_ev_buf_->begin(); it != shuffle_ev_buf_->end(); ++it) {
    int target_rank = (*shuffle_parti_)[(*it)->destination()];
    out_bufs[target_rank*max_buf_size + index[target_rank] ] = **it;
    ++index[target_rank];
  }

  /* shuffle events */
  std::vector<event<App> > in_bufs(com_world_->size()*max_buf_size,
                                   event<App>());
  boost::mpi::all_to_all(*com_world_, out_bufs, max_buf_size, in_bufs);

  /* insert in_bufs to ret */
  for (int rank = 0; rank < com_world_->size(); ++rank) {
    for (int i = 0; i < max_buf_size; ++i) {
      if (in_bufs[rank*max_buf_size + i].id() == -1) break;
      shuffle_ev_ret_->push_back(
          boost::make_shared<event<App> >(event<App>(in_bufs[rank + i])));
    }
  }
}

template<class App>
void mpi_collection<App>::shuffle_what_if() {
  /* calculate  maximum size of buffers */
  std::vector<int> msg_num_to_rank(com_world_->size(), 0);
  for (auto it = shuffle_wi_buf_->begin(); it != shuffle_wi_buf_->end(); ++it) {
    int target_rank = (*shuffle_parti_)[(*it)->lp_id_];
    ++msg_num_to_rank[target_rank];
  }
  int max_buf_size
      = *std::max_element(msg_num_to_rank.begin(), msg_num_to_rank.end());
  max_buf_size
      = boost::mpi::all_reduce(*com_world_, max_buf_size,
                               boost::mpi::maximum<int>());

  /* make events vectors, where events for rank i are in out_bufs[i] */
  std::vector<what_if<App> > out_bufs(com_world_->size()*max_buf_size,
                                    what_if<App>());
  std::vector<int> index(com_world_->size(), 0);
  for (auto it = shuffle_wi_buf_->begin();it != shuffle_wi_buf_->end(); ++it) {
    int target_rank = (*shuffle_parti_)[(*it)->lp_id_];
    out_bufs[target_rank*max_buf_size + index[target_rank] ] = **it;
    ++index[target_rank];
  }

  /* shuffle events */
  std::vector<what_if<App> > in_bufs(com_world_->size()*max_buf_size,
                                     what_if<App>());
  boost::mpi::all_to_all(*com_world_, out_bufs, max_buf_size, in_bufs);

  /* insert in_bufs to ret */
  for (int rank = 0; rank < com_world_->size(); ++rank) {
    for (int i = 0; i < max_buf_size; ++i) {
      if (in_bufs[rank*max_buf_size + i].lp_id_ == -1) break;
      shuffle_wi_ret_->push_back(
          boost::make_shared<what_if<App> >(what_if<App>(in_bufs[rank + i])));
    }
  }
};

} /* namespace scalesim */

#endif /* SCALESIM_COM_MPI_COLLECTION_HPP_ */
