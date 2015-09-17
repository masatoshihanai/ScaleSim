/*
 * trafficsim_app_test.cc
 *
 *  Created on: Feb 10, 2015
 *      Author: masahanai
 */
#include <string>
#include "gtest/gtest.h"
#include "scalesim/simulation.hpp"

#define main foo // for compiling main_trafficsim.cc by gtest
#include "trafficsim/main_trafficsim.cc"

namespace {

TEST (traffic_sim, ring_trip) {
//  //TODO L test of traffic simulation
//  TRAFFIC_PARTITION_PATH = "input/traffic/test/ring/partition/trafficgraph.part.4";
//  TRAFFIC_MAP_PATH = "input/traffic/test/ring/rd.csv";
//  TRIP_PATH = "input/traffic/test/ring/trip.csv";
//  std::map<long, boost::shared_ptr<std::string> > ret;
//  ret = scalesim::runner<traffic_sim>::test();
//  for (std::map<long, boost::shared_ptr<std::string> >::iterator it = ret.begin();
//      it != ret.end(); ++it) {
//    std::cout << *it->second << std::endl;
//  }
}

} // namespace
