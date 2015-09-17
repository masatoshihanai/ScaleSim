/*
 * flags.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#ifndef SCALESIM_UTIL_FLAGS_HPP_
#define SCALESIM_UTIL_FLAGS_HPP_

#include <gflags/gflags.h>

/* flags from command line */
DEFINE_bool(diff_init, false, "Initial Execution of Differential Simulation");
DEFINE_bool(diff_repeat, false, "Repeat Execution of Differential Simulation");
DEFINE_string(sim_id, "0", "Execution ID for Differential Simulation");
DEFINE_string(store_dir, "", "Directory PATH for Event/State Stores");
DEFINE_string(store_ip, "", "IP address for Event/State Stores");

#endif /* SCALESIM_UTIL_FLAGS_HPP_ */
