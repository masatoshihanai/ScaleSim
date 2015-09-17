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
    /* send event */
    requests_.push_back(comm_world_->isend(send_events_->begin()->first,
                                           0,
                                           *send_events_->begin()->second));
    send_events_->erase(send_events_->begin());
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
  std::vector<boost::mpi::request> requests_;
  std::vector<event<App> > rec_events_;
 public:
  void init(const boost::mpi::communicator* comm_world) {
    comm_world_ = comm_world;
  };
  void async_receive();
  void check_receive(
      const boost::function<void(const ev_ptr<App>&)>& call_back_f);
};

template<class App>
void mpi_receiver<App>::async_receive() {
  if (comm_world_->iprobe()) {
    rec_events_.push_back(event<App>());
    requests_.push_back(comm_world_->irecv(boost::mpi::any_source,
                                           0,
                                           rec_events_.back()));
  }
};

template<class App>
void mpi_receiver<App>::check_receive(
    const boost::function<void(const ev_ptr<App>&)>& call_back_f) {
  typename std::vector<boost::mpi::request>::iterator it;
  while ((it = boost::mpi::test_some(requests_.begin(), requests_.end()) )
             != requests_.end()) {
    int index = it - requests_.begin();
    ev_ptr<App> receive_event
        = boost::make_shared<event<App> >(rec_events_[index]);
    call_back_f(receive_event);
    timestamp tmstmp_(receive_event->receive_time(), receive_event->id());
    mpi_gsync<App>::instance()->update_local(tmstmp_);

    /* count white transit message */
    if (rec_events_[index].is_white()) {
      mpi_gsync<App>::instance()->decrement_transit_conut();
    }
    rec_events_.erase(rec_events_.begin() + index);
    requests_.erase(it);
  }
};

} /* namepsace scalesim */
#endif /* SCALESIM_COM_MPI_SENDER_RECEIVER_HPP_ */
