/*
 * phold.hpp
 *
 *  Copyright (c) 2016 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#ifndef PHOLD_PHOLD_HPP_
#define PHOLD_PHOLD_HPP_

#include <string>
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include "scalesim/simulation.hpp"
#include "scalesim/util.hpp"

/**
 * Benchmark program based on phold
 *
 * References
 *   "Warp Speed: Executing Time Warp on 1,966,080 Cores" (PADS '14)
 *     - http://dl.acm.org/citation.cfm?id=2486134
 *   "On Deciding Between Conservative and Optimistic Approaches
 *    on Massively Parallel Platforms" (WSC '10)
 *     - http://dl.acm.org/citation.cfm?id=2433588
 *   "Scalable Time Warp on Blue Gene Supercomputers" (PADS '09)
 *     - http://dl.acm.org/citation.cfm?id=1577971
 */
class phold: public scalesim::application {
 public:
  class Event: public scalesim::sim_event {
   friend class phold;
   private:
    long id_; long src_id_; long dst_id_;
    long receive_time_; long send_time_; int num_hops_;
    mutable scalesim::sim_event_base base_;
   public:
    Event(): id_(-1), src_id_(-1), dst_id_(-1),
             receive_time_(-1), send_time_(-1), num_hops_(0) {};
    virtual ~Event(){};
    Event(long event_id, long src_id, long dst_id,
          long receive_time, long send_time, int num_hops):
            id_(event_id), src_id_(src_id), dst_id_(dst_id),
            receive_time_(receive_time), send_time_(send_time),
            num_hops_(num_hops) {};
    Event(const Event& event) {
      id_ = event.id_; src_id_ = event.src_id_; dst_id_ = event.dst_id_;
      receive_time_ = event.receive_time_; send_time_ = event.send_time_;
      num_hops_ = event.num_hops_;
      base_ = event.base_;
    };

   public:
    scalesim::sim_event_base* base() const { return &base_; };
    long id() const { return id_; };
    long source() const { return src_id_; };
    long destination() const { return dst_id_; };
    bool end() const { return true; };
    long receive_time() const { return receive_time_; };
    long send_time() const { return send_time_; };
    int size() const { return sizeof(*this); };

    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & id_;
      ar & src_id_;
      ar & dst_id_;
      ar & receive_time_;
      ar & send_time_;
      ar & num_hops_;
      ar & base_;
    }
  }; /* class event */

  class State : public scalesim::sim_state {
    friend class phold;
   private:
    long id_;
   public:
    State(): id_(-1) {};
    virtual ~State() {};
    State(long id): id_(id){};
    long id() const { return id_; }
    int size() const { return sizeof(*this); }
    void out_put() const {
      std::cout << "state id: " << id_ << std::endl;
    };
    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & id_;
    }
  }; /* class State */

 public:
  /*
   * Return finish time of the simulation
   */
  static long finish_time();

  /*
   * Initiation function for application.
   * It is invoked before all initiation functions.
   */
  void init();

  /*
   * Initiation function for partition and index.
   * Partition format:
   *   - type: boost::shared_ptr<std::std::vector<long> >
   *   - value: Nth value represents a rank number in ID=N
   * Index format:
   *   - type: boost::shared_ptr<boost::unordered_multimap<long, long> >
   *   - key: rank number
   *   - value: IDs in this rank
   */
  std::pair<parti_ptr, parti_indx_ptr> init_partition_index(int rank, int rank_size);

  /*
   * Initiation function for events.
   * Initiated events are shuffled to each starting point after this function.
   * Thus it is ok to just read in whatever way.
   * For example, use modulo operator based on rank_id and rank_size.
   */
  void init_events(ev_vec<phold>& ret,
                   const int rank,
                   const int rank_size);

  /*
   * Initiation function for states.
   * Initiated states are NOT shuffled after this function.
   * Thus you have to initiate only states in this rank, based on partition.
   */
  void init_states_in_this_rank(st_vec<phold>& new_state,
                                const int rank,
                                const int rank_size,
                                parti_ptr partition);

  /*
   * Initiation function for what_if events.
   * Initiated events are shuffled to each starting point after this function.
   * Thus it is ok to just read in whatever way.
   */
  void init_what_if(
      std::vector<boost::shared_ptr<const scalesim::what_if<phold> > >& ret,
      const int rank,
      const int rank_size);

  /*
   * Event handling function.
   * The arguments (receive_event, state) are previous value in the simulation.
   * The return value (optional<pair<ev_pst, st_ptr> >) should include
   * new event and state based on the arguments and your simulation models.
   *
   * If there are no new event and state generated, return empty option.
   */
  boost::optional<std::pair<std::vector<ev_ptr<phold> >, st_ptr<phold> > >
  event_handler(ev_ptr<phold> receive_event, st_ptr<phold> state);
};

#endif /* PHOLD_PHOLD_HPP_ */
