/*
 * sender_receiver.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_COM_MPI_SENDER_RECEIVER_HPP_
#define SCALESIM_COM_MPI_SENDER_RECEIVER_HPP_

#include <boost/make_shared.hpp>
#include "scalesim/com/mpi/global_sync.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template<class App>
class mpi_sender {
 private:
  mpi_sender(const mpi_sender&);
  void operator=(const mpi_sender&);
 public:
  mpi_sender(){};
  virtual ~mpi_sender() {
    if (buffer_) { delete buffer_; }
    if (send_events_) { delete send_events_; }
  };
 private:
  std::vector<std::pair<int, ev_ptr<App> > >* buffer_; boost::mutex buf_mutex_;
  std::vector<std::pair<int, ev_ptr<App> > >* send_events_;
  const boost::mpi::communicator* comm_world_;
  std::vector<boost::mpi::request> requests_;
 public:
  void init(const boost::mpi::communicator* communicator);
  void buffer_sendevent(const int rank, const ev_ptr<App>& event);
  void async_send();
  void check_send();
  void reset();
 private:
  void change_buffer();
};

template<class App>
void mpi_sender<App>::init(const boost::mpi::communicator* communicator) {
  buffer_ = new std::vector<std::pair<int, ev_ptr<App> > >;
  send_events_ = new std::vector<std::pair<int, ev_ptr<App> > >;
  comm_world_ = communicator;
}

template<class App>
void mpi_sender<App>::buffer_sendevent(const int rank,
                                       const ev_ptr<App>& event) {
  boost::lock_guard<boost::mutex> guard(buf_mutex_);
  /* count sending white message for global sync. */
  if (mpi_gsync<App>::instance()->is_red()) {
    /* is red */
    event->change_red();
    timestamp tmstmp_(event->send_time(), event->id());
    mpi_gsync<App>::instance()->update_local(tmstmp_);
  } else {
    /* is white */
    event->change_white();
    mpi_gsync<App>::instance()->increment_transit_conut();
  }
  buffer_->push_back(std::pair<int, ev_ptr<App> >(rank, event));
};

template<class App>
void mpi_sender<App>::async_send() {
  if (send_events_->empty()) {
    change_buffer();
  }
  while (!send_events_->empty()) {
    stopwatch::instance("PartiToPartiCommunication")->start();
    /* send event */
    requests_.push_back(comm_world_->isend(send_events_->begin()->first,
                                           0,
                                           *send_events_->begin()->second));
    send_events_->erase(send_events_->begin());
    stopwatch::instance("PartiToPartiCommunication")->stop();
  }
};

template<class App>
void mpi_sender<App>::check_send() {
  auto it = boost::mpi::test_some(requests_.begin(), requests_.end());
  if (it != requests_.end()) {
    requests_.erase(it);
  }
};

template<class App>
void mpi_sender<App>::reset() {
  buffer_->clear();
  send_events_->clear();
};

template<class App>
void mpi_sender<App>::change_buffer() {
  boost::lock_guard<boost::mutex> guard(buf_mutex_);
  delete send_events_;
  send_events_ = buffer_;
  buffer_ = new std::vector<std::pair<int, ev_ptr<App> > >;
};

template<class App>
class mpi_receiver {
 private:
  mpi_receiver(const mpi_receiver&);
  void operator=(const mpi_receiver&);
 public:
  mpi_receiver(){};
  virtual ~mpi_receiver(){};
 private:
  const boost::mpi::communicator* comm_world_;

 public:
  void init(const boost::mpi::communicator* comm_world)
    { comm_world_ = comm_world; };
  void receive(const boost::function<void(const ev_ptr<App>&)>& call_back_f);
};

template<class App>
void mpi_receiver<App>::receive(
    const boost::function<void(const ev_ptr<App>&)>& call_back_f) {
  while (comm_world_->iprobe()) {
    stopwatch::instance("PartiToPartiCommunication")->start();
    event<App> event_;
    comm_world_->recv(boost::mpi::any_source, 0, event_);
    ev_ptr<App> receive_event
        = boost::make_shared<event<App> >(event_);
    call_back_f(receive_event);
    timestamp tmstmp_(receive_event->receive_time(), receive_event->id());
    mpi_gsync<App>::instance()->update_local(tmstmp_);

    /* count white transit message */
    if (event_.is_white()) {
      mpi_gsync<App>::instance()->decrement_transit_conut();
    }
    stopwatch::instance("PartiToPartiCommunication")->stop();
  }
};

} /* namepsace scalesim */
#endif /* SCALESIM_COM_MPI_SENDER_RECEIVER_HPP_ */
