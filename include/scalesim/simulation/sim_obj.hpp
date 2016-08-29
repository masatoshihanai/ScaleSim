/*
 * sim_obj.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_SIMULATION_SIM_OBJ_HPP_
#define SCALESIM_SIMULATION_SIM_OBJ_HPP_

#include <boost/serialization/serialization.hpp>
#include "scalesim/util/type.hpp"

namespace scalesim {

class sim_event_base {
 public:
  sim_event_base(): is_white_(true), is_cancel_(false){};
  virtual ~sim_event_base(){};
  sim_event_base(const sim_event_base& base) {
    is_white_ = base.is_white_;
    is_cancel_ = base.is_cancel_;
  }
 private:
  mutable bool is_white_;
  mutable bool is_cancel_;
 public:
  bool is_white() const { return is_white_; };
  bool is_red() const { return !is_white_; };
  void change_white() const { is_white_ = true; };
  void change_red() const { is_white_ = false; };
  bool is_cancel() const { return is_cancel_; }
  void change_cancel() const { is_cancel_ = true; };
 private:
  friend class boost::serialization::access;
 template<class Archive>
  void serialize(Archive& ar, unsigned int version) {
   ar & is_white_;
   ar & is_cancel_;
 };
};

class sim_event {
 protected:
  sim_event() {};
  virtual ~sim_event() {};
 public:
  virtual long id() const { return -1; };
  virtual long source() const { return -1; };
  virtual long destination() const { return -1; };
  virtual bool end() const { return false; };
  virtual long receive_time() const { return -1; };
  virtual long send_time() const { return -1; };
  /* return byte size of the message */
  virtual int size() const { return -1; };

  virtual sim_event_base* base() const = 0;
  bool is_white() const { return this->base()->is_white(); };
  bool is_red() const { return this->base()->is_red(); };
  void change_white() const { this->base()->change_white(); };
  void change_red() const { this->base()->change_red(); };
  void change_cancel() const {this->base()->change_cancel(); };
  bool is_cancel() const { return this->base()->is_cancel(); };
  void result_out() const {
    std::cout << "RE" /* REsult */
              << "," << this->id()
              << "," << this->source()
              << "," << this->send_time()
              << "," << this->destination()
              << "," << this->receive_time();

    if (this->source() == -1) { std::cout << ",START"; }
    else if (this->end()) { std::cout << ",END"; }
    std::cout << std::endl;
  };

  bool operator==(const sim_event& right) const {
    if (id() == right.id() &&
        destination() == right.destination() &&
        receive_time() == right.receive_time() &&
        send_time() == right.send_time()) {
      return true;
    }
    return false;
  };
};

class sim_state {
 public:
  sim_state() {};
  virtual ~sim_state(){};
 public:
  virtual long id() const = 0;
  virtual int size() const { return -1; };
  void result_out() const {};
};

template<class App>
class what_if {
 public:
  what_if(): lp_id_(-1), time_(-1), delete_event_id_(-1){};
  virtual ~what_if() {};
  what_if(const what_if& value) {
    lp_id_ = value.lp_id_;
    time_ = value.time_;
    update_state_ = value.update_state_;
    add_event_ = value.add_event_;
    delete_event_id_ = value.delete_event_id_;
  }
  what_if(const long lp_id, const long time): lp_id_(lp_id), time_(time), delete_event_id_(-1) {};
  what_if(const long lp_id, const long time, const state<App>& update_state):
      lp_id_(lp_id), time_(time), update_state_(update_state), delete_event_id_(-1) {};
  what_if(const long lp_id, const long time, const event<App>& add_event):
      lp_id_(lp_id), time_(time), add_event_(add_event), delete_event_id_(-1) {};
  what_if(const long lp_id, const long time, const long delete_event_id):
      lp_id_(lp_id), time_(time), delete_event_id_(delete_event_id) {};
 public:
  long lp_id_;
  long time_;
  state<App> update_state_;
  event<App> add_event_;
  long delete_event_id_;
 private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, unsigned int version) {
    ar & lp_id_;
    ar & time_;
    ar & update_state_;
    ar & add_event_;
    ar & delete_event_id_;
  };
};

} /* namespace scalesim */

#endif /* SCALESIM_SIMULATION_SIM_OBJ_HPP_ */
