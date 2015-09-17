/*
 * trafficsim_app_test.cc
 *
 *  Created on: Jan 14, 2015
 *      Author: masahanai
 */
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"
#include "scalesim/simulation.hpp"

#define main foo // for compiling main_trafficsim.cc by gtest
#include "trafficsim/main_trafficsim.cc"

typedef boost::shared_ptr<const traffic_sim::Event> vehicle_ptr;
typedef boost::shared_ptr<const traffic_sim::State> state_ptr;

//TEST (trafficsim_test, arrival_time) {
//  // !!! This test should depend on the modeling of vehicle moving !!!
//  traffic_sim simulation;
//  const long vehicle_id = 0; long path_tracks[MAX_PATH_LENGTH];
//  long depature_time = 0; const long track_length = 20;
//  long track_counter = 0;
//  for (int i = 0; i < track_length; ++i) {path_tracks[i] = i;}
//
//  long cp_id = 0;
//  std::vector<long> destination_crosspoint;
//  destination_crosspoint.push_back(1);
//  std::vector<long> speed_limit;
//  speed_limit.push_back(60);
//  std::vector<int> num_lanes;
//  num_lanes.push_back(1);
//  std::vector<long> road_length;
//  road_length.push_back(100);
//
//  state_ptr cp(new traffic_sim::State(cp_id, destination_crosspoint,
//      speed_limit, num_lanes, road_length));
//
//  long arrival_time = 10; // starting time of the source cross point
//  vehicle_ptr vehicle(new traffic_sim::Event(vehicle_id, arrival_time,
//                            depature_time, path_tracks, track_counter, track_length));
//
//  boost::optional<std::pair<vehicle_ptr, state_ptr> > ret
//    = simulation.event_handler(vehicle, cp);
//  vehicle_ptr ret_vehicle = ret->first;
//  long expected = road_length[0] * 60 * 60 / speed_limit[0] / 1000 + 10;
//  // TODO more sophisticated model
//  EXPECT_EQ(expected, ret_vehicle->receive_time()); // arrival time of the destination cross point.
//}
//
//TEST (trafficsim_test, processing_10000_events) {
//  traffic_sim simulation;
//  long path_tracks[MAX_PATH_LENGTH]; long depature_time = 0;
//  const long track_length = 20; long track_counter = 0;
//  for (int i = 0; i < track_length; ++i) {path_tracks[i] = i;}
//  long cp_id = 0;
//  std::vector<long> destination_crosspoint;
//  destination_crosspoint.push_back(1);
//  std::vector<long> speed_limit;
//  speed_limit.push_back(1);
//  std::vector<int> num_lanes;
//  num_lanes.push_back(1);
//  std::vector<long> road_length;
//  road_length.push_back(100);
//
//  state_ptr cp(new traffic_sim::State(cp_id, destination_crosspoint,
//      speed_limit, num_lanes, road_length));
//
//  std::vector<vehicle_ptr> vehicles(10000);
//  for (int i = 0; i < 10000; ++i) {
//    vehicles[i] = vehicle_ptr(new traffic_sim::Event(i, i, 0,
//        path_tracks, track_counter, track_length));
//  }
//  for (int i = 0; i < 10000; ++i) {
//    EXPECT_EQ(i, simulation.event_handler(vehicles[i], cp)->first->id());
//  }
//}
//
//TEST(trafficsim_test, vehicle_id) {
//  traffic_sim simulation;
//  vehicle_ptr default_vehicle(new traffic_sim::Event());
//  EXPECT_EQ(-1, default_vehicle->id());
//
//  long arrival_time = 0; long depature_time = 0; long position = 0;
//  long path_tracks[MAX_PATH_LENGTH]; const long track_length = 20; long track_counter = 0;
//  for (int i = 0; i < track_length; ++i) {path_tracks[i] = i;}
//
//  std::vector<long> destination_crosspoint;
//  destination_crosspoint.push_back(1);
//  std::vector<long> speed_limit;
//  speed_limit.push_back(1);
//  std::vector<int> num_lanes;
//  num_lanes.push_back(1);
//  std::vector<long> road_length;
//  road_length.push_back(100);
//
//  const long vehicle_id = 0;
//  vehicle_ptr vehicle(new traffic_sim::Event(vehicle_id, arrival_time,
//                            depature_time, path_tracks, track_counter, track_length));
//
//  long cp_id = 0;
//  state_ptr cp(new traffic_sim::State(cp_id, destination_crosspoint,
//      speed_limit, num_lanes, road_length));
//
//  boost::optional<std::pair<vehicle_ptr, state_ptr> >
//    ret = simulation.event_handler(vehicle, cp);
//  EXPECT_EQ(0, ret->first->id());
//}
//
//TEST(trafficsim_test, departure_time) {
//  traffic_sim simulation;
//  const long vehicle_id = 0; long position = 0; long path_tracks[MAX_PATH_LENGTH];
//  long depature_time = 0; const long track_length = 20; long track_counter = 0;
//  for (int i = 0; i < track_length; ++i) {path_tracks[i] = i;}
//
//  long cp_id = 0;
//  std::vector<long> destination_crosspoint;
//  destination_crosspoint.push_back(1);
//  std::vector<long> speed_limit;
//  speed_limit.push_back(1);
//  std::vector<int> num_lanes;
//  num_lanes.push_back(1);
//  std::vector<long> road_length;
//  road_length.push_back(100);
//
//  state_ptr cp(new traffic_sim::State(cp_id, destination_crosspoint,
//      speed_limit, num_lanes, road_length));
//
//
//  long arrival_time = 10;
//  vehicle_ptr vehicle(new traffic_sim::Event(vehicle_id, arrival_time,
//                            depature_time, path_tracks, track_counter, track_length));
//
//  boost::optional<std::pair<vehicle_ptr, state_ptr> >
//    ret = simulation.event_handler(vehicle, cp);
//  vehicle_ptr ret_vehicle = ret->first;
//  EXPECT_EQ(10, ret_vehicle->send_time()); // departure time
//}
//
//TEST (trafficsim_test, track_crosspoints) {
//  traffic_sim simulation;
//  const long vehicle_id = 0; long position = 0;
//  long arrival_time = 10; long depature_time = 15;
//
//  std::vector<long> speed_limit;
//  speed_limit.push_back(1);
//  std::vector<int> num_lanes;
//  num_lanes.push_back(1);
//  std::vector<long> road_length;
//  road_length.push_back(100);
//
//  long path_tracks[MAX_PATH_LENGTH];
//  const long track_length = 10; long track_counter = 0;
//  for (int i = 0; i < track_length; ++i) {
//    path_tracks[i] = i;
//  }
//  vehicle_ptr vehicle0(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            0, track_length));
//  vehicle_ptr vehicle1(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            1, track_length));
//  vehicle_ptr vehicle2(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            2, track_length));
//  vehicle_ptr vehicle3(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            3, track_length));
//  vehicle_ptr vehicle4(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            4, track_length));
//  vehicle_ptr vehicle5(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            5, track_length));
//  vehicle_ptr vehicle6(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            6, track_length));
//  vehicle_ptr vehicle7(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            7, track_length));
//  vehicle_ptr vehicle8(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            8, track_length));
//  vehicle_ptr vehicle9(new traffic_sim::Event(vehicle_id, arrival_time, depature_time, path_tracks,
//                            9, track_length));
//
//  long cp_id_0 = 0;
//  std::vector<long> destination_crosspoint_1;
//  destination_crosspoint_1.push_back(1);
//  state_ptr cp0(new traffic_sim::State(cp_id_0, destination_crosspoint_1,
//      speed_limit, num_lanes, road_length));
//  long cp_id_1 = 1;
//  std::vector<long> destination_crosspoint_2;
//  destination_crosspoint_2.push_back(2);
//  state_ptr cp1(new traffic_sim::State(cp_id_1, destination_crosspoint_2,
//      speed_limit, num_lanes, road_length));
//  long cp_id_2 = 2;
//  std::vector<long> destination_crosspoint_3;
//  destination_crosspoint_3.push_back(3);
//  state_ptr cp2(new traffic_sim::State(cp_id_2, destination_crosspoint_3,
//      speed_limit, num_lanes, road_length));
//  long cp_id_3 = 3;
//  std::vector<long> destination_crosspoint_4;
//  destination_crosspoint_4.push_back(4);
//  state_ptr cp3(new traffic_sim::State(cp_id_3, destination_crosspoint_4,
//      speed_limit, num_lanes, road_length));
//  long cp_id_4 = 4;
//  std::vector<long> destination_crosspoint_5;
//  destination_crosspoint_5.push_back(5);
//  state_ptr cp4(new traffic_sim::State(cp_id_4, destination_crosspoint_5,
//      speed_limit, num_lanes, road_length));
//  long cp_id_5 = 5;
//  std::vector<long> destination_crosspoint_6;
//  destination_crosspoint_6.push_back(6);
//  state_ptr cp5(new traffic_sim::State(cp_id_5, destination_crosspoint_6,
//      speed_limit, num_lanes, road_length));
//  long cp_id_6 = 6;
//  std::vector<long> destination_crosspoint_7;
//  destination_crosspoint_7.push_back(7);
//  state_ptr cp6(new traffic_sim::State(cp_id_6, destination_crosspoint_7,
//      speed_limit, num_lanes, road_length));
//  long cp_id_7 = 7;
//  std::vector<long> destination_crosspoint_8;
//  destination_crosspoint_8.push_back(8);
//  state_ptr cp7(new traffic_sim::State(cp_id_7, destination_crosspoint_8,
//      speed_limit, num_lanes, road_length));
//  long cp_id_8 = 8;
//  std::vector<long> destination_crosspoint_9;
//  destination_crosspoint_9.push_back(9);
//  state_ptr cp8(new traffic_sim::State(cp_id_8, destination_crosspoint_9,
//      speed_limit, num_lanes, road_length));
//
//  EXPECT_EQ(1, simulation.event_handler(vehicle0, cp0)->first->destination());
//  EXPECT_EQ(2, simulation.event_handler(vehicle1, cp1)->first->destination());
//  EXPECT_EQ(3, simulation.event_handler(vehicle2, cp2)->first->destination());
//  EXPECT_EQ(4, simulation.event_handler(vehicle3, cp3)->first->destination());
//  EXPECT_EQ(5, simulation.event_handler(vehicle4, cp4)->first->destination());
//  EXPECT_EQ(6, simulation.event_handler(vehicle5, cp5)->first->destination());
//  EXPECT_EQ(7, simulation.event_handler(vehicle6, cp6)->first->destination());
//  EXPECT_EQ(8, simulation.event_handler(vehicle7, cp7)->first->destination());
//  EXPECT_EQ(9, simulation.event_handler(vehicle8, cp8)->first->destination());
//}

//TEST (trafficsim_test, input_event) { //todo implement
//  traffic_sim simulation;
//  TRIP_PATH = "input/traffic/test/ring/trip.csv";
//  // TP,0,0,1,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0
//  // TP,1,0,10,1,2,3,4,5,6,7
//  ev_vec<traffic_sim> vehicles;
//  simulation.parse_event(0, 1);
//  EXPECT_EQ(0, (*vehicles)[0].id());
//  EXPECT_EQ(1, (*vehicles)[0].send_time());
//  EXPECT_EQ(0, (*vehicles)[0].destination());
//
//  EXPECT_EQ(1, (*vehicles)[1].id());
//  EXPECT_EQ(10, (*vehicles)[1].send_time());
//  EXPECT_EQ(1, (*vehicles)[1].destination());
//}
//
//
//TEST (trafficsim_test, input_ring) {
//  traffic_sim simulation;
//  TRAFFIC_MAP_PATH = "input/traffic/test/ring/rd.csv";
//  //  R,0,0,1,10,100,1
//  //  R,1,1,2,10,100,1
//  //  R,2,2,3,10,100,1
//  //  R,3,3,4,10,100,1
//  //  R,4,4,5,10,100,1
//  //  R,5,5,6,10,100,1
//  //  R,6,6,7,10,100,1
//  //  R,7,7,8,10,100,1
//  //  R,8,8,9,10,100,1
//  //  R,9,9,0,10,100,1
//  boost::shared_ptr<std::vector<long> > partition(new std::vector<long>);
//  for (int i = 0; i < 10; ++i) {
//    partition->push_back(0);
//  }
//  boost::shared_ptr<std::vector<state_ptr> > cps;
//  simulation.parse_state(cps, 0, 1, partition);
//  for (int i = 0; i < 10; ++i) {
//    EXPECT_EQ(i, (*cps)[i]->id());
//    EXPECT_EQ((i + 1) % 10, (*cps)[i]->destinations_[0]);
//    EXPECT_EQ(10, (*cps)[i]->speed_limit_[0]);
//    EXPECT_EQ(100, (*cps)[i]->road_length_[0]);
//    EXPECT_EQ(1, (*cps)[i]->num_lanes_[0]);
//  }
//}
//
//TEST (trafficsim_test, input_grid) {
//  traffic_sim simulation;
//  TRAFFIC_MAP_PATH = "input/traffic/test/grid/rd.csv";
//  //  R,0,0,1,10,100,1;R,1,0,3,10,100,1
//  //  R,2,1,2,10,100,1;R,3,1,4,10,100,1
//  //  R,4,2,5,10,100,1
//  //  R,5,3,6,10,100,1;R,6,3,4,10,100,1b
//  //  R,7,4,7,10,100,1;R,8,4,5,10,100,1
//  //  R,9,5,8,10,100,1
//  //  R,10,6,7,10,100,1
//  //  R,11,7,8,10,100,1
//  boost::shared_ptr<std::vector<long> > partition(new std::vector<long>);
//  for (int i = 0; i < 10; ++i) {
//    partition->push_back(0);
//  }
//  boost::shared_ptr<std::vector<state_ptr> > states;
//  simulation.parse_state(states, 0, 1, partition);
//  for (int i = 0; i < 8; ++i) {
//    EXPECT_EQ(i, (*states)[i]->id());
//  }
//  // cp 0
//  EXPECT_EQ(1, (*states)[0]->destinations_[0]);
//  EXPECT_EQ(3, (*states)[0]->destinations_[1]);
//  // cp 1
//  EXPECT_EQ(2, (*states)[1]->destinations_[0]);
//  EXPECT_EQ(4, (*states)[1]->destinations_[1]);
//}
//
