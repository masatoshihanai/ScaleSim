/*
 * main_phold.cc
 *
 *  Copyright (c) 2016 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#include "scalesim/simulation.hpp"
#include "phold.hpp"

int main(int argc, char* argv[]) {
  scalesim::runner<phold> runner_;
  runner_.init_main(&argc, &argv);

  /* Application Parameters */
  NUM_LP = std::stol(argv[1]);
  NUM_INIT_MSG = std::stol(argv[2]);
  REMOTE_COM_RATIO = std::stod(argv[3]);
  LAMBDA = std::stod(argv[4]);
  NUM_WHAT_IF = std::stoi(argv[5]);

  /* System Parameters */
  RANDOM_SEED = 1;
  GSYNC_INTERVAL = 10;
  LP_INTERVAL = 1;
  GTW_CUT_INTERVAL = std::stoi(argv[6]);

  runner_.run();
  return 0;
};
