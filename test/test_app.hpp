/*
 * test_app.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef TEST_APP_CC_
#define TEST_APP_CC_

#include "scalesim/simulation.hpp"

namespace scalesim {

class test_app: public scalesim::application {
 public:
  typedef boost::shared_ptr<std::vector<long> > vec_ptr;
  typedef boost::shared_ptr<boost::unordered_multimap<long, long> > hash_ptr;
  std::pair<vec_ptr, hash_ptr> partition() {
    std::pair<vec_ptr, hash_ptr>
      ret(vec_ptr(new std::vector<long>()),
          hash_ptr(new boost::unordered_multimap<long, long>()));
    scalesim::graph_reader reader;
    reader.read("input/traffic/test/ring/partition/trafficgraph.part.4",
        ret.first, ret.second);
    return ret;
  };

  class Event : public scalesim::sim_event {
   private:
    long id_; long send_time_; long receive_time_; long destination_; long array_[100];
    mutable scalesim::sim_event_base base_;
   public:
    Event(): send_time_(0), receive_time_(0), destination_(0), id_(-1) {
      for (int i = 0; i < 100; ++i) { array_[i] = 0; }
    };
    Event(long id, long send_time, long receive_time, long destination, int array[100]):
        id_(id), send_time_(send_time), receive_time_(receive_time), destination_(destination){
      for (int i = 0; i < 100; ++i) { array_[i] = array[i]; }
    };
    Event(const Event& event): id_(event.id_), send_time_(event.send_time_),
        receive_time_(event.receive_time_), destination_(event.destination_),
        base_(event.base_) {
      for (int i = 0; i < 100; ++i) { array_[i] = event.array_[i]; }
    };
    long id() const { return id_; };
    long send_time() const { return send_time_; };
    long receive_time() const { return receive_time_; };
    long destination() const { return destination_; };

    scalesim::sim_event_base* base() const { return &base_; };
    long get_vec(int i) const { return array_[i]; };
    bool operator==(const Event& right) const {
      if (id() == right.id() &&
          destination() == right.destination() &&
          receive_time() == right.receive_time() &&
          send_time() == right.send_time()) {
        return true;
      }
      return false;
    };
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar& send_time_; ar & receive_time_; ar & destination_;
      ar & id_; ar & array_; ar & base_;
    }
  }; /* class Event */

  class State : public scalesim::sim_state {
   public:
    long id_;
   public:
    State(): id_(-1){};
    virtual ~State() {};
    explicit State(long id): id_(id) {};
    long id() const { return id_; }
    void out_put() const { std::cout << "state id: " << id_ << std::endl; };
    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & id_;
    }
  }; /* class State */

}; /* class test_app */

} /* namespace scalesim */

#endif /* TEST_APP_CC_ */
