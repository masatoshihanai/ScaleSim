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

#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include "scalesim/util.hpp"

/**
 * The traffic modeling is based on the paper.
 * http://kalper.net/kp/publications/docs/rcveh-pads08.pdf
 */

static std::string TRAFFIC_PARTITION_PATH;
static std::string TRAFFIC_MAP_PATH;
static std::string SCENARIO_PATH;

class traffic_sim: public scalesim::application {
 public:
  /* an event represents a vehicle */
  class Event: public scalesim::sim_event {
    friend class traffic_sim;
   private:
    long vehicle_id;
    long arrival_time; /* second */
    long departure_time; /* second */
    long source_;
    std::vector<long> destinations_; /* lp id */
    mutable scalesim::sim_event_base base_;
   public:
    Event(): vehicle_id(-1), arrival_time(-1), departure_time(-1), source_(-1) {};
    virtual ~Event(){};
    Event(long vehicle_id_,
          long arrival_time_,
          long departure_time_,
          long source_,
          std::vector<long> destinations):
            vehicle_id(vehicle_id_),
            arrival_time(arrival_time_),
            departure_time(departure_time_),
            source_(source_) {
      for (auto it = destinations.begin(); it != destinations.end(); ++it) {
        destinations_.push_back(*it);
      }
    };
    Event(const Event& event) {
      vehicle_id = event.vehicle_id;
      arrival_time = event.arrival_time;
      departure_time = event.departure_time;
      source_ = event.source_;
      for (auto it = event.destinations_.begin();
          it != event.destinations_.end(); ++it) {
        destinations_.push_back(*it);
      }
      base_ = event.base_;
    };

   public:
    scalesim::sim_event_base* base() const { return &base_; };
    long id() const { return vehicle_id; };
    long source() const { return source_; };
    long destination() const { return destinations_[0]; };
    bool end() const { return (destinations_.size() == 1); };
    long receive_time() const { return arrival_time; };
    long send_time() const { return departure_time; };
    int size() const {
      return sizeof(*this) + sizeof(long)*destinations_.size();
    };

    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & vehicle_id;
      ar & arrival_time;
      ar & departure_time;
      ar & source_;
      ar & destinations_;
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
    State(): id_(-1) {};
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
    int size() const {
      return sizeof(*this) +
             sizeof(long) * (destinations_.size() +
                             speed_limit_.size() +
                             num_lanes_.size() +
                             road_length_.size());
    }
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
  std::pair<parti_ptr, parti_indx_ptr> init_partition_index(int rank_size);

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
  boost::optional<std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> > >
  event_handler(ev_ptr<traffic_sim> receive_event, st_ptr<traffic_sim> state);
};

class traffic_reader {
 private:
  traffic_reader(const traffic_reader&);
  void operator=(const traffic_reader&);
 private:
  traffic_reader() {};
  virtual ~traffic_reader() {};
 public:
  static void graph_read(parti_ptr ret_partition,
                         parti_indx_ptr ret_partition_index,
                         const std::string& file_path);

  static void road_read(st_vec<traffic_sim>& ret,
                        int rank,
                        int rank_size,
                        parti_ptr partition,
                        const std::string& file_path);

  static void trip_read(ev_vec<traffic_sim>& ret,
                        int rank,
                        int rank_size,
                        const std::string& file_path);

  static void what_if_read(std::vector<boost::shared_ptr<
      const scalesim::what_if<traffic_sim> > >& ret,
                           const int rank,
                           const int rank_size,
                           const std::string& file_path);
}; /* class traffic_reader */

long traffic_sim::finish_time() {
//  return 500;
//  return 3600;      /*  1 hours */
  return 10800;   /*  3 hours */
//  return 21600;   /*  6 hours */
//  return 86400;   /* 24 hours */
//  return std::numeric_limits<long>::max();
};

void traffic_sim::init() {};

std::pair<parti_ptr, parti_indx_ptr> traffic_sim::init_partition_index(int rank_size) {
  auto partition_ = boost::make_shared<std::vector<long> >(std::vector<long>());
  auto index_ = boost::make_shared<boost::unordered_multimap<long, long> >(
      boost::unordered_multimap<long, long>());
  traffic_reader::graph_read(partition_, index_, TRAFFIC_PARTITION_PATH);
  return std::pair<parti_ptr, parti_indx_ptr>(partition_, index_);
};

void traffic_sim::init_events(ev_vec<traffic_sim>& ret,
                              const int rank,
                              const int rank_size) {
  traffic_reader::trip_read(ret, rank, rank_size, SCENARIO_PATH);
};

void traffic_sim::init_states_in_this_rank(st_vec<traffic_sim>& new_state,
                                           const int rank,
                                           const int rank_size,
                                           parti_ptr partition) {
  traffic_reader::road_read(new_state, rank, rank_size, partition, TRAFFIC_MAP_PATH);
};

void traffic_sim::init_what_if(
    std::vector<boost::shared_ptr<const scalesim::what_if<traffic_sim> > >& ret,
    const int rank,
    const int rank_size) {
  traffic_reader::what_if_read(ret, rank, rank_size, SCENARIO_PATH);
};

boost::optional<std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> > >
traffic_sim::event_handler(ev_ptr<traffic_sim> receive_event, st_ptr<traffic_sim> state) {
  /* define destinations */
  std::vector<long> new_destinations_;
  auto tracks_it_ = receive_event->destinations_.begin();
  ++tracks_it_;

  if (tracks_it_ == receive_event->destinations_.end()) {
    return boost::optional<std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> > >();
  }

  while (tracks_it_ != receive_event->destinations_.end()) {
    new_destinations_.push_back(*tracks_it_);
    ++tracks_it_;
  }

  if (new_destinations_.empty()) {
    return boost::optional<std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> > >();
  }

  /* calculate reaching time to next junction */
  auto it = std::find(state->destinations_.begin(),
                      state->destinations_.end(),
                      new_destinations_.front());

  if (it == state->destinations_.end()) {
    return boost::optional<std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> > >();
  }
  DLOG_ASSERT(it != state->destinations_.end())
      << " No road from: " << state->id() << " to " << new_destinations_.front()
      << "\n See vehicle: " << receive_event->id() << "'s tracks";

  long new_arrival_time = 0;
  int i = it - state->destinations_.begin();
  if (state->speed_limit_[i] != 0) {
    new_arrival_time
        = state->road_length_[i] * 60 * 60 / state->speed_limit_[i] / 1000
            + receive_event->arrival_time + 5;
  } else {
    new_arrival_time = state->road_length_[i] * 60 * 60
                       + receive_event->arrival_time;
  }
  long new_source_ = receive_event->destinations_.front();

  long new_depature_time = receive_event->arrival_time;

  std::vector<ev_ptr<traffic_sim> > new_event;
  new_event.push_back(boost::make_shared<traffic_sim::Event>(
      traffic_sim::Event(receive_event->vehicle_id,
                         new_arrival_time,
                         new_depature_time,
                         new_source_,
                         new_destinations_)));

  st_ptr<traffic_sim> new_state = boost::make_shared<traffic_sim::State>(
      traffic_sim::State(state->id_,
                         state->destinations_,
                         state->speed_limit_,
                         state->num_lanes_,
                         state->road_length_));

  return boost::optional<std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> > > (
      std::pair<std::vector<ev_ptr<traffic_sim> >, st_ptr<traffic_sim> >(new_event, new_state));
};


void traffic_reader::graph_read(parti_ptr ret_partition,
                                parti_indx_ptr ret_partition_index,
                                const std::string& file_path) {
  std::ifstream ifstream(file_path.c_str());
  if (ifstream.fail()) {
    ifstream.close();
    return;
  }
  std::string line;
  long id = 0;
  while (getline(ifstream, line)) {
    long partitionNum = atol(line.c_str());
    ret_partition_index->insert(std::pair<long, long>(partitionNum, id));
    ret_partition->push_back(partitionNum);
    id++;
  }
  ifstream.close();
};

void traffic_reader::road_read(st_vec<traffic_sim>& ret,
                               int rank,
                               int rank_size,
                               parti_ptr partition,
                               const std::string& file_path) {
  std::ifstream ifstream(file_path.c_str());
  if (ifstream.fail()) {
    ifstream.close();
    return;
  }
  std::string line;
  long id = 0;
  while (getline(ifstream, line)) {
    if ((*partition)[id]%rank_size == rank) {
      std::vector<std::string> cp;
      boost::split(cp, line, boost::is_any_of(";"));
      long state_id;
      std::vector<long> destinations;
      std::vector<long> speed_limit;
      std::vector<int> num_lanes;
      std::vector<long> road_length;
      for (auto road_it = cp.begin(); road_it != cp.end(); ++road_it) {
        std::vector<std::string> road;
        boost::split(road, *road_it, boost::is_any_of(","));
        state_id = atol(road[2].c_str());
        destinations.push_back(atol(road[3].c_str()));
        speed_limit.push_back(atol(road[4].c_str()));
        num_lanes.push_back(atol(road[6].c_str()));
        if (atol(road[5].c_str()) != 0) {
          road_length.push_back(atol(road[5].c_str()));
        } else {
          road_length.push_back((1 + atol(road[5].c_str())));
        }
      }
      ret.push_back(
          boost::make_shared<state<traffic_sim> >(
              state<traffic_sim>(state_id,
                                 destinations,
                                 speed_limit,
                                 num_lanes,
                                 road_length)));
    }
    id++;
  } /* while (getline(ifstream, line)) */
  ifstream.close();
  return;
}; /* load_read() */

void traffic_reader::trip_read(ev_vec<traffic_sim>& ret,
                               int rank,
                               int rank_size,
                               const std::string& file_path) {
  std::ifstream ifstream(file_path.c_str());
  if (ifstream.fail()) {
    ifstream.close();
    return;
  }
  long id = 0;
  std::string line;
  while (getline(ifstream, line)) {
    if (id % rank_size == rank) {
      std::vector<std::string> ev_str_;
      boost::split(ev_str_, line, boost::is_any_of(","));
      long vehicle_id = atol(ev_str_[1].c_str());
      long arrival_time = atol(ev_str_[3].c_str());
      long departure_time = atol(ev_str_[3].c_str());
//      int track_counter = 0;
      int track_length = ev_str_.size() - 4;
      std::vector<long> tracks_;
      if (track_length == 0) {
        std::cout << "test " << std::endl;
      }
      for (int i = 0; i < track_length; ++i) {
        tracks_.push_back(atol(ev_str_[i + 4].c_str()));
      }
      ret.push_back(
          boost::make_shared<event<traffic_sim> >(
              event<traffic_sim>(vehicle_id,
                                 arrival_time,
                                 departure_time,
                                 -1,
                                 tracks_)));
    }
    ++id;
  } /* while (getline(ifstream, line)) */
  ifstream.close();
  return;
}; /* trip_read() */

void traffic_reader::what_if_read(std::vector<boost::shared_ptr<const scalesim::what_if<traffic_sim> > >& ret,
                                  const int rank,
                                  const int rank_size,
                                  const std::string& file_path) {
  /* Read file */
  std::ifstream ifstream_(file_path);
  if (ifstream_.fail()) {
    ifstream_.close();
    LOG(INFO) << " Opening file " << file_path << " fails. Check file.";
    exit(1);
  }

  /* Read lines */
  long i = 0;
  std::string line;
  while (std::getline(ifstream_, line)) {
    if (i % rank_size != rank) {
      ++i;
      continue;
    }
    if (line.compare(0, 2, "SC") == 0) {
      /*
       * Query type is state change (SC).
       * Input format is like that.
       *   SC,{lp id},{time};{lp format};...;..;..
       * ex)
       *   SC,10,555;R,0,10,1,10,100,2;R,1,10,1,10,200,1;R,3,10,3,10,200,100
       */
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(";"));

      /* read lp id & time */
      std::vector<std::string> id_time_;
      boost::split(id_time_, vec_[0], boost::is_any_of(","));
      long lp_id = atol(id_time_[1].c_str());
      long time_ = atol(id_time_[2].c_str());

      /* read roads */
      std::vector<long> destinations;
      std::vector<long> speed_limit;
      std::vector<int> num_lanes;
      std::vector<long> rd_length;
      auto road_it = vec_.begin();
      ++road_it;
      for (;road_it != vec_.end(); ++road_it) {
        std::vector<std::string> road;
        boost::split(road, *road_it, boost::is_any_of(","));
        destinations.push_back(atol(road[3].c_str()));
        speed_limit.push_back(atol(road[4].c_str()));
        num_lanes.push_back(atol(road[6].c_str()));
        if (atol(road[5].c_str()) != 0) {
          rd_length.push_back(atol(road[5].c_str()));
        } else {
          rd_length.push_back(1 + atol(road[5].c_str()));
        }
      }

      /* initiate what_if query */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id,time_,
                                             state<traffic_sim>(lp_id,
                                                                destinations,
                                                                speed_limit,
                                                                num_lanes,
                                                                rd_length)));

      /* push back this what if query */
      ret.push_back(wh_if_);
    } else if (line.compare(0, 2, "AE") == 0) {
      /*
       * Query type is add event (AE).
       * Input format is like that.
       *   AE,{event id},0,{Deptime},{1st cp},{2nd cp},...
       * ex)
       *   AE,0,0,1,0,1,2,3,4,5
       */
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(","));

      /* read lp id and time */
      long lp_id_ = atol(vec_[4].c_str());
      long time_ = atol(vec_[3].c_str());

      /* read adding event */
      long vehicle_id_ = atol(vec_[1].c_str());
      long arrival_time_ = atol(vec_[3].c_str());
      long departure_time_ = atol(vec_[3].c_str());
      int length_ = vec_.size() - 4;
      std::vector<long> tracks_;
      for (int i = 0; i < length_; ++i) {
        tracks_.push_back(atol(vec_[i + 4].c_str()));
      }

      /* initiate adding event */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id_, time_,
                                             event<traffic_sim>(vehicle_id_,
                                                                arrival_time_,
                                                                departure_time_,
                                                                -1, /* source id */
                                                                tracks_)));

      /* push back to return */
      ret.push_back(wh_if_);
    } else if (line.compare(0,2,"DE") == 0) {
      /*
       * Query type is delete event (DE).
       * Input format is like that.
       *   DE, {lp id}, {time}, {event id}
       *   RE, {event id}, {source id}, {send time}, {destination id}, {receive time}
       * ex)
       *   DE,999,10,555
       *   RE,2,2,94,3,135
       */
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(","));
      /* read lp id, time, event id */
      long lp_id_ = atol(vec_[1].c_str());
      long time_ = atol(vec_[2].c_str());
      long ev_id_ = atol(vec_[3].c_str());

      /* initiate what-if query */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id_, time_, ev_id_));

      ret.push_back(wh_if_);
    } else if (line.compare(0,2,"RE") == 0) {
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(","));
      /* read lp id, time, event id */
      long lp_id_ = atol(vec_[4].c_str());
      long time_ = atol(vec_[5].c_str());
      long ev_id_ = atol(vec_[1].c_str());

      /* initiate what-if query */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id_, time_, ev_id_));

      ret.push_back(wh_if_);
    }
    ++i;
  } /* while (std::getline(ifstream_, line)) */
}; /* what_if_read*/

#endif /* TRAFFICSIM_TRAFFIC_SIM_HPP_ */
