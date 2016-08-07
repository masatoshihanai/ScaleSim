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
  std::vector<std::vector<event<App> >* >* buffer_; boost::mutex buf_mutex_;
  std::vector<std::vector<event<App> >* >* send_events_;
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
  comm_world_ = communicator;
  buffer_ = new std::vector<std::vector<event<App> >* >(communicator->size());
  send_events_ = new std::vector<std::vector<event<App> >* >(communicator->size());

  for (int i = 0; i < communicator->size(); ++i) {
    (*buffer_)[i] = new std::vector<event<App> >();
    (*send_events_)[i] = new std::vector<event<App> >();
    std::cout << send_events_->size() << std::endl;
  }
}

template<class App>
void mpi_sender<App>::buffer_sendevent(const int rank,
                                       const ev_ptr<App>& event) {
  /* Buffer to Send */
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
  (*buffer_)[rank]->push_back(*event);
};

template<class App>
void mpi_sender<App>::async_send() {
  bool change_buffer_flag = true;
  for (int i = 0; i < comm_world_->size(); ++i) {
    if (!((*send_events_)[i]->empty())) {
      change_buffer_flag = false;
      break;
    }
  }
  if (change_buffer_flag) change_buffer();

  stopwatch::instance("PartiToPartiCommunication")->start();
  for (int i = 0; i < comm_world_->size(); ++i) {
    if ((*send_events_)[i]->size() == 0) continue;
    requests_.push_back(comm_world_->isend(i, 0, *(*send_events_)[i]));
    (*send_events_)[i]->clear();
  }
  stopwatch::instance("PartiToPartiCommunication")->stop();
};

template<class App>
void mpi_sender<App>::check_send() {
  auto it = boost::mpi::test_some(requests_.begin(), requests_.end());
  while (it != requests_.end()) {
    requests_.erase(it);
    it = boost::mpi::test_some(requests_.begin(), requests_.end());
  }
};

template<class App>
void mpi_sender<App>::reset() {
  for (int i = 0; i < comm_world_->size(); ++i) {
    (*buffer_)[i]->clear();
    (*send_events_)[i]->clear();
  }
};

template<class App>
void mpi_sender<App>::change_buffer() {
  boost::lock_guard<boost::mutex> guard(buf_mutex_);
  for (int i = 0; i < comm_world_->size(); ++i) {
    delete (*send_events_)[i];
    (*send_events_)[i] = NULL;
  }
  delete send_events_;
  send_events_ = buffer_;
  buffer_ = new std::vector<std::vector<event<App> >* >(comm_world_->size());
  for (int i = 0; i < comm_world_->size(); ++i) {
    (*buffer_)[i] = new std::vector<event<App> >();
  }
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
  stopwatch::instance("PartiToPartiCommunication")->start();
  while (comm_world_->iprobe()) {
    std::vector<event<App> > events_;
    comm_world_->recv(boost::mpi::any_source, 0, events_);
    for (auto it = events_.begin(); it != events_.end(); ++it) {
      ev_ptr<App> receive_event = boost::make_shared<event<App> >(*it);
      call_back_f(receive_event);
      timestamp tmstmp_(receive_event->receive_time(), receive_event->id());
      mpi_gsync<App>::instance()->update_local(tmstmp_);
      /* count white transit message */
      if (receive_event->is_white()) {
        mpi_gsync<App>::instance()->decrement_transit_conut();
      }
    }
  }
  stopwatch::instance("PartiToPartiCommunication")->stop();
};

} /* namepsace scalesim */
#endif /* SCALESIM_COM_MPI_SENDER_RECEIVER_HPP_ */
