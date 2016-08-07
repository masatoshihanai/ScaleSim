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

  NUM_LP = std::stol(argv[1]);
  NUM_INIT_MSG = std::stol(argv[2]);
  REMOTE_COM_RATIO = std::stod(argv[3]);
  LAMBDA = std::stod(argv[4]);
  // TODO
  NUM_WHAT_IF = 1;
  GSYNC_INTERVAL = 10;
  LP_INTERVAL = 4;
  GTW_CUT_INTERVAL = 50;

  runner_.run();
  return 0;
};
