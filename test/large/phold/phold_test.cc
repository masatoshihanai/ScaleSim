/*
 * phold_test.cc
 *
 *  Copyright (c) 2016 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "phold/phold.hpp"

#define main foo /* for compiling main_phold.cc by gtest */
#include "phold/main_phold.cc"

namespace {

TEST (phold, ordering_check) {
  EXPECT_TRUE(false);
}

TEST (phold, init_partition) {
  phold simulation;
  const int rank_size = 10;
  auto actual = simulation.init_partition_index(rank_size);

  /* Check partition */
  int id = 0;
  for (auto itr = actual.first->begin(); itr != actual.first->end(); ++itr) {
    EXPECT_EQ(*itr, id % rank_size);
    ++id;
  }

  /* Check index */
  for (int partition = 0; partition != rank_size; ++partition) {
    for (auto itr = actual.second->find(partition); itr->first != partition; ++itr) {
      EXPECT_EQ(itr->second % rank_size, partition);
    }
  }
}

TEST (phold, init_event) {
  ev_vec<phold> initial_events;
  phold simulation;
  simulation.init_events(initial_events, 0, 1);

  EXPECT_EQ(16, initial_events.size());
}

TEST (phold, init_states) {
  EXPECT_TRUE(false);
}

TEST (phold, init_what_if) {
  EXPECT_TRUE(false);
}

TEST (phold, event_hander) {
  EXPECT_TRUE(false);
}

} /* namespace */

#undef main

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(*argv);
  google::InstallFailureSignalHandler();

  return RUN_ALL_TESTS();
}
