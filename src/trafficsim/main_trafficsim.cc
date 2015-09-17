/*
 * main_trafficsim.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "scalesim/simulation.hpp"
#include "scalesim/util.hpp"
#include <glog/logging.h>

//FIXME !!! in the case over 1000, message truncate has occur !!!
#define MAX_PATH_LENGTH 200
#define MAX_NUM_ROAD_PER_CP 20

using namespace std;

static string TRAFFIC_PARTITION_PATH;
static string TRAFFIC_MAP_PATH;
static string SCENARIO_PATH;

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
  typedef boost::shared_ptr<const traffic_sim::Event> event_ptr;
  typedef boost::shared_ptr<vector<event_ptr> > events_ptr;

  class State : public scalesim::sim_state { // a state represents a cross point with outgoing road
    friend class traffic_sim;
   private:
    long id_;
    vector<long> destinations_;
    vector<long> speed_limit_; // km / hour
    vector<int> num_lanes_;
    vector<long> road_length_; // m
   public:
    State(): id_(-1){};
    virtual ~State() {};
    State(long id, vector<long> destiantions,
        vector<long> speed_limit,
        vector<int> num_lanes,
        vector<long> road_length) {
      id_ = id;
      destinations_ = destiantions;
      speed_limit_ = speed_limit;
      num_lanes_ = num_lanes;
      road_length_ = road_length;
    };
    long id() const { return id_; }
    void out_put() const {
      cout << "state id: " << id_ << endl;
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
  }; // class state
  typedef boost::shared_ptr<const traffic_sim::State> state_ptr;
  typedef boost::shared_ptr<vector<state_ptr> > state_vec;

 private:
  parti_ptr partition_;
  parti_indx_ptr index_;

 public:
  static long finish_time() {
    return 1000;
  //  return 10800;
  //  return 21600; // 6hours
  //  return 1000000;
  //  return std::numeric_limits<long>::max();
  };
  static string partition_path() {
    return TRAFFIC_PARTITION_PATH;
  };

  void init() {
    partition_ = parti_ptr(new vector<long>()),
    index_ = parti_indx_ptr(new boost::unordered_multimap<long, long>());
    scalesim::graph_reader reader;
    reader.read(TRAFFIC_PARTITION_PATH, partition_, index_);
  };

  pair<parti_ptr, parti_indx_ptr> partition_index() {
    return pair<parti_ptr, parti_indx_ptr>(partition_, index_);
  };

  void init_events(ev_vec<traffic_sim>& ret,int rank, int rank_size) {
    scalesim::event_reader reader;
    reader.read(SCENARIO_PATH, rank, rank_size);
    for (vector<string>::iterator it = reader.events().begin();
        it != reader.events().end(); ++it) {
      vector<string> line;
      boost::split(line, *it, boost::is_any_of(","));
      long vehicle_id = atol(line[1].c_str());
      long arrival_time = atol(line[3].c_str());
      long departure_time = atol(line[3].c_str());
      int track_counter = 0;
      int track_length = line.size() - 4;
      long path_tracks[MAX_PATH_LENGTH];
      if (track_length > MAX_PATH_LENGTH) {
        cerr << "!! the vehicle: " << vehicle_id << " has track_length " << track_length;
        cerr << " tracks, which is bigger than MAX_PATH_LENGTH " << MAX_PATH_LENGTH << endl;
        track_length = MAX_PATH_LENGTH;
      }
      for (int i = 0; i < track_length; ++i) {
        path_tracks[i] = atol(line[i + 4].c_str());
      }
      ret.push_back(boost::make_shared<traffic_sim::Event>
          (traffic_sim::Event(vehicle_id, arrival_time,
          departure_time, path_tracks, track_counter, track_length)));
    }
  };

  void init_states_in_this_rank(st_vec<traffic_sim>& new_state, int rank, int rank_size) {
    parse_state(new_state, rank, rank_size, this->partition_);
  };

  void parse_state(st_vec<traffic_sim>& new_state, int rank, int rank_size, parti_ptr parti) {
    scalesim::state_reader reader;
    reader.read(TRAFFIC_MAP_PATH, rank, rank_size, parti);
    for (vector<pair<long, string> >::iterator cp_it = reader.states().begin();
        cp_it != reader.states().end(); ++cp_it) {
      vector<string> cp;
      boost::split(cp, cp_it->second, boost::is_any_of(";"));
      long id;
      vector<long> destinations(MAX_NUM_ROAD_PER_CP);
      vector<long> speed_limit(MAX_NUM_ROAD_PER_CP);
      vector<int> num_lanes(MAX_NUM_ROAD_PER_CP);
      vector<long> road_length(MAX_NUM_ROAD_PER_CP);
      int road_index = 0;
      for (vector<string>::iterator road_it = cp.begin();
          road_it != cp.end(); ++road_it) {
        if (road_index > MAX_NUM_ROAD_PER_CP) {
          cerr << "# of roads per junction is BIGGER than MAX_NUM_ROAD_PER_CP in " << id << std::endl;
        }
        vector<string> road;
        boost::split(road, *road_it, boost::is_any_of(","));
        id = atol(road[2].c_str());
        destinations[road_index] = atol(road[3].c_str());
        speed_limit[road_index] = atol(road[4].c_str());
        num_lanes[road_index] = atol(road[6].c_str());
        if (atol(road[5].c_str()) != 0) {
          road_length[road_index] = atol(road[5].c_str());
        } else {
          road_length[road_index] = (1 + atol(road[5].c_str()));
        }
        ++road_index;
      }
      new_state.push_back(state_ptr(new State(id, destinations,
          speed_limit, num_lanes,road_length)));
    }
  };

  void init_what_if(
      vector<boost::shared_ptr<const scalesim::what_if<traffic_sim> > >& ret,
      const int rank, const int rank_size) {
    /* Read file */
    std::ifstream ifstream_(SCENARIO_PATH);
    if (ifstream_.fail()) {
      ifstream_.close();
      LOG(INFO) << " Opening file " << SCENARIO_PATH << " fails. Check file.";
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
        vector<string> vec_;
        boost::split(vec_, line, boost::is_any_of(";"));

        /* read lp id & time */
        vector<string> id_time_;
        boost::split(id_time_, vec_[0], boost::is_any_of(","));
        long lp_id = atol(id_time_[1].c_str());
        long time_ = atol(id_time_[2].c_str());

        /* read roads */
        vector<long> destinations(MAX_NUM_ROAD_PER_CP);
        vector<long> speed_limit(MAX_NUM_ROAD_PER_CP);
        vector<int> num_lanes(MAX_NUM_ROAD_PER_CP);
        vector<long> rd_length(MAX_NUM_ROAD_PER_CP);
        int road_index = 0;
        auto road_it = vec_.begin();
        ++road_it;
        for (;road_it != vec_.end(); ++road_it) {
          if (road_index > MAX_NUM_ROAD_PER_CP) {
            cerr << "# of roads per junction is BIGGER than MAX_NUM_ROAD_PER_CP in " << lp_id << std::endl;
          }
          vector<string> road;
          boost::split(road, *road_it, boost::is_any_of(","));
          destinations[road_index] = atol(road[3].c_str());
          speed_limit[road_index] = atol(road[4].c_str());
          num_lanes[road_index] = atol(road[6].c_str());
          if (atol(road[5].c_str()) != 0) {
            rd_length[road_index] = atol(road[5].c_str());
          } else {
            rd_length[road_index] = (1 + atol(road[5].c_str()));
          }
          ++road_index;
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
        vector<string> vec_;
        boost::split(vec_, line, boost::is_any_of(","));

        /* read lp id and time */
        long lp_id_ = atol(vec_[4].c_str());
        long time_ = atol(vec_[3].c_str());

        /* read adding event */
        long vehicle_id_ = atol(vec_[1].c_str());
        long arrival_time_ = atol(vec_[3].c_str());
        long departure_time_ = atol(vec_[3].c_str());
        int track_counter_ = 0;
        int length_ = vec_.size() - 4;
        long path_tracks_[MAX_PATH_LENGTH];
        if (length_ > MAX_PATH_LENGTH) {
          cerr << "!! the vehicle: " << vehicle_id_
               << " has track_length " << length_
               << " tracks, which is bigger than MAX_PATH_LENGTH "
               << MAX_PATH_LENGTH << endl;
          length_ = MAX_PATH_LENGTH;
        }
        for (int i = 0; i < length_; ++i) {
          path_tracks_[i] = atol(vec_[i + 4].c_str());
        }

        /* initiate adding event */
        boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id_, time_,
                                             event<traffic_sim>(vehicle_id_,
                                                                arrival_time_,
                                                                departure_time_,
                                                                path_tracks_,
                                                                track_counter_,
                                                                length_)));

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
        vector<string> vec_;
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
        vector<string> vec_;
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
  };

  boost::optional<pair<event_ptr, state_ptr> >
  event_handler(event_ptr receive_event, state_ptr state) {
    long new_path_tracks[MAX_PATH_LENGTH];
    int new_track_counter = receive_event->track_counter + 1;
    int new_track_length = std::min(receive_event->track_length, MAX_PATH_LENGTH);
    for (int i = 0; i < new_track_length; ++i) {
      new_path_tracks[i] = receive_event->path_tracks[i];
    }
    long new_depature_time = receive_event->arrival_time;

    long new_arrival_time;
    //find the iterator of road to the destination
    vector<long> new_destinations = state->destinations_;
    for(vector<long>::iterator it = new_destinations.begin();
        it != new_destinations.end(); ++it) {
      // road destination == vehicle destination
      if (*it == new_path_tracks[new_track_counter]) {
        int i = distance(new_destinations.begin(), it);
        if (state->speed_limit_[i] != 0) {
          new_arrival_time
            = state->road_length_[i] * 60 * 60 / state->speed_limit_[i] / 1000
              + receive_event->arrival_time + 5;
        } else {
          new_arrival_time = state->road_length_[i] * 60 * 60
                             + receive_event->arrival_time;
        }
        break;
      }
    }

    event_ptr new_event(new traffic_sim::Event(receive_event->vehicle_id,
      new_arrival_time, new_depature_time, new_path_tracks,
      new_track_counter, new_track_length));

    state_ptr new_state(new traffic_sim::State(state->id_,
      state->destinations_, state->speed_limit_, state->num_lanes_,
      state->road_length_));

    if (new_track_counter >= new_track_length) {
      return boost::optional<pair<event_ptr, state_ptr> >();
    }

    return boost::optional<pair<event_ptr, state_ptr> > (
        pair<event_ptr, state_ptr>(new_event, new_state));
  };
};

int main (int argc, char* argv[]) {
  scalesim::runner<traffic_sim>::init_main(&argc, &argv);

  /* set input file path */
  TRAFFIC_PARTITION_PATH = argv[1];
  TRAFFIC_MAP_PATH = argv[2];
  SCENARIO_PATH = argv[3];

  scalesim::runner<traffic_sim>::run();
  return 0;
};
