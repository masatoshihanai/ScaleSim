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
#include "scalesim/util.hpp"

namespace scalesim {

class application {
 private:
  application(const application&);
  void operator=(const application&);

 protected:
  application(){};
  virtual ~application(){};

 public:
  /*
   * Interval between global synchronizations
   * The return value means # of processing logical processes before next global synchronization.
   */
  static int gsync_interval() { return 100; };

  /*
   * Interval between switching logical processes in event processing.
   * The return value means # of dequeue events before switching next logical process.
   */
  static int switch_lp_interval() { return 5; };

  /*
   * Interval between second cut and next first cut
   * Too small value causes wrong results because of events in send buffer.
   */
  static int global_cut_interval() { return 20; };

  /*
   * # of thread for processing simulation event
   * In general, the effective # is (# of processors) per Node.
   */
  static int num_thr () { return boost::thread::physical_concurrency() - 1; };

  /*
   * Simulation finishes when global time >= finish_time().
   */
  static long finish_time() { return std::numeric_limits<long>::max(); };
};

} /* namespace scalesim */

#endif /* SCALESIM_SIMULATION_APPLICATION_HPP_ */
