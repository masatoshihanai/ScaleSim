/*
 * main_trafficsim.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "scalesim/simulation.hpp"
#include "traffic_sim.hpp"

int main (int argc, char* argv[]) {
  scalesim::runner<traffic_sim> runner_;
  runner_.init_main(&argc, &argv);

  /* set input file path */
  TRAFFIC_PARTITION_PATH = argv[1];
  TRAFFIC_MAP_PATH = argv[2];
  SCENARIO_PATH = argv[3];

  runner_.run();
  return 0;
};
