/*
 * traffic_sim.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#ifndef TRAFFICSIM_TRAFFIC_SIM_HPP_
#define TRAFFICSIM_TRAFFIC_SIM_HPP_

#include <string>
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include "scalesim/util.hpp"

//FIXME !!! in the case over 1000, message truncate has occur !!!
#define MAX_PATH_LENGTH 200
#define MAX_NUM_ROAD_PER_CP 20

/**
 * The traffic modeling is based on the paper.
 * http://kalper.net/kp/publications/docs/rcveh-pads08.pdf
 */
class traffic_sim: public scalesim::application {
 public:
  /* an event represents a vehicle */
  class Event: public scalesim::sim_event {
    friend class traffic_sim;
   private:
    long vehicle_id;
    long arrival_time; /* second */
    long departure_time; /* second */
    int track_counter;
    int track_length;
    long path_tracks[MAX_PATH_LENGTH];
    mutable scalesim::sim_event_base base_;
   public:
    Event(): vehicle_id(-1), arrival_time(-1), departure_time(-1),
             track_counter(0), track_length(0) { };
    virtual ~Event(){};
    Event(long vehicle_id_,
          long arrival_time_,
          long departure_time_,
          long path_tracks_[MAX_PATH_LENGTH],
          int track_counter_,
          int track_length_):
            vehicle_id(vehicle_id_),
            arrival_time(arrival_time_),
            departure_time(departure_time_),
            track_counter(track_counter_),
            track_length(track_length_) {
      std::memcpy(path_tracks, path_tracks_, sizeof(long) * track_length_);
    };
    Event(const Event& event) {
      vehicle_id = event.vehicle_id;
      arrival_time = event.arrival_time;
      departure_time = event.departure_time;
      track_counter = event.track_counter;
      track_length = event.track_length;
      std::memcpy(path_tracks, event.path_tracks,
                  sizeof(long) * event.track_length);
      base_ = event.base_;
    };

   public:
    scalesim::sim_event_base* base() const { return &base_; };
    long id() const { return vehicle_id; };
    long source() const {
      return track_counter > 0 ? path_tracks[track_counter - 1] : -1;
    };
    long destination() const { return path_tracks[track_counter]; };
    long end() const { return path_tracks[track_length - 1]; };
    long receive_time() const { return arrival_time; };
    long send_time() const { return departure_time; };

    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & vehicle_id;
      ar & arrival_time;
      ar & departure_time;
      ar & track_counter;
      ar & track_length;
      ar & path_tracks;
      ar & base_;
    }
  }; /* class event */

  /* a state represents a cross point with outgoing road */
  class State : public scalesim::sim_state {
     friend class traffic_sim;
   private:
    long id_;
    std::vector<long> destinations_;
    std::vector<long> speed_limit_; /* km per hour */
    std::vector<int> num_lanes_;
    std::vector<long> road_length_; /* m */
   public:
    State(): id_(-1){};
    virtual ~State() {};
    State(long id, std::vector<long> destiantions,
        std::vector<long> speed_limit,
        std::vector<int> num_lanes,
        std::vector<long> road_length) {
      id_ = id;
      destinations_ = destiantions;
      speed_limit_ = speed_limit;
      num_lanes_ = num_lanes;
      road_length_ = road_length;
    };
    long id() const { return id_; }
    void out_put() const {
      std::cout << "state id: " << id_ << std::endl;
    };

    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & id_;
      ar & destinations_;
      ar & speed_limit_;
      ar & num_lanes_;
      ar & road_length_;
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
  std::pair<parti_ptr, parti_indx_ptr> init_partition_index();

  /*
   * Initiation function for events.
   * Initiated events are shuffled to each starting point after this function.
   * Thus it is ok to just read in whatever way.
   * For example, use modulo operator based on rank_id and rank_size.
   */
  void init_events(ev_vec<traffic_sim>& ret,
                   const int rank,
                   const int rank_size);

  /*
   * Initiation function for states.
   * Initiated states are NOT shuffled after this function.
   * Thus you have to initiate only states in this rank, based on partition.
   */
  void init_states_in_this_rank(st_vec<traffic_sim>& new_state,
                                const int rank,
                                const int rank_size,
                                parti_ptr partition);

  /*
   * Initiation function for what_if events.
   * Initiated events are shuffled to each starting point after this function.
   * Thus it is ok to just read in whatever way.
   */
  void init_what_if(
      std::vector<boost::shared_ptr<const scalesim::what_if<traffic_sim> > >& ret,
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
  boost::optional<std::pair<ev_ptr<traffic_sim>, st_ptr<traffic_sim> > >
  event_handler(ev_ptr<traffic_sim> receive_event, st_ptr<traffic_sim> state);
};

#endif /* TRAFFICSIM_TRAFFIC_SIM_HPP_ */
