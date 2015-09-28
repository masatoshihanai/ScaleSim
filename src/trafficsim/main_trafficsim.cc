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
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include "scalesim/simulation.hpp"
#include "scalesim/util.hpp"
#include "traffic_sim.hpp"
#include "traffic_reader.hpp"

using namespace std;

static string TRAFFIC_PARTITION_PATH;
static string TRAFFIC_MAP_PATH;
static string SCENARIO_PATH;

long traffic_sim::finish_time() {
//  return 500;
  return 3600;      /*  1 hours */
//  return 10800;   /*  3 hours */
//  return 21600;   /*  6 hours */
//  return 86400;   /* 24 hours */
//  return std::numeric_limits<long>::max();
};

void traffic_sim::init() {};

pair<parti_ptr, parti_indx_ptr> traffic_sim::init_partition_index() {
  auto partition_ = boost::make_shared<vector<long> >(vector<long>());
  auto index_ = boost::make_shared<boost::unordered_multimap<long, long> >(
                   boost::unordered_multimap<long, long>());
  traffic_reader::graph_read(partition_, index_, TRAFFIC_PARTITION_PATH);
  return pair<parti_ptr, parti_indx_ptr>(partition_, index_);
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
         vector<boost::shared_ptr<const scalesim::what_if<traffic_sim> > >& ret,
         const int rank,
         const int rank_size) {
  traffic_reader::what_if_read(ret, rank, rank_size, SCENARIO_PATH);
};

boost::optional<pair<ev_ptr<traffic_sim>, st_ptr<traffic_sim> > >
traffic_sim::event_handler(ev_ptr<traffic_sim> receive_event,
                           st_ptr<traffic_sim> state) {
  long new_path_tracks[MAX_PATH_LENGTH];
  int new_track_counter = receive_event->track_counter + 1;
  int new_track_length = std::min(receive_event->track_length, MAX_PATH_LENGTH);
  for (int i = 0; i < new_track_length; ++i) {
    new_path_tracks[i] = receive_event->path_tracks[i];
  }
  long new_depature_time = receive_event->arrival_time;

  long new_arrival_time;

  /* find the iterator of road to the destination */
  vector<long> new_destinations = state->destinations_;
  for(auto it = new_destinations.begin();
           it != new_destinations.end(); ++it) {
    /* road destination == vehicle destination */
    if (*it == new_path_tracks[new_track_counter]) {

      /* calculate reaching time to next junction */
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

  ev_ptr<traffic_sim> new_event = boost::make_shared<traffic_sim::Event>(
      traffic_sim::Event(receive_event->vehicle_id,
                         new_arrival_time,
                         new_depature_time,
                         new_path_tracks,
                         new_track_counter,
                         new_track_length));

  st_ptr<traffic_sim> new_state = boost::make_shared<traffic_sim::State>(
      traffic_sim::State(state->id_,
                         state->destinations_,
                         state->speed_limit_,
                         state->num_lanes_,
                         state->road_length_));

  if (new_track_counter >= new_track_length) {
    return boost::optional<pair<ev_ptr<traffic_sim>, st_ptr<traffic_sim> > >();
  }

  return boost::optional<pair<ev_ptr<traffic_sim>, st_ptr<traffic_sim> > > (
      pair<ev_ptr<traffic_sim>, st_ptr<traffic_sim> >(new_event, new_state));
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
