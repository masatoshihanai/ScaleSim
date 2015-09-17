/*
 * application.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_SIMULATION_APPLICATION_HPP_
#define SCALESIM_SIMULATION_APPLICATION_HPP_

#include <limits>

namespace scalesim {

class application {
 public:
  /*
   * Interval between communications
   * The comm_interval means # of processing events between communications.
   */
  static int comm_interval() { return 3; };

  /*
   * Interval between second cut and next first cut
   * The global_interval is a interval size between global virtual time.
   * Too small value causes wrong results because of events in send buffer.
   */
  static int gsync_interval() { return 5; };

  /*
   * # of thread pool size
   * The effective # is (# of processors) + 1.
   * For instance, when a machine has 4 cores, 5 (4 core + 1) is effective.
   */
  static int thr_pool_size () { return 1; };


  /*
   * Simulation finishes when global time >= finish_time().
   */
  static long finish_time() { return std::numeric_limits<long>::max(); };
};

} /* namespace scalesim */

#endif /* SCALESIM_SIMULATION_APPLICATION_HPP_ */
