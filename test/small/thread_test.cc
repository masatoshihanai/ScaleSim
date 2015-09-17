/*
 * thread_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "gtest/gtest.h"
#include "scalesim/util.hpp"


class sim_runner_small: public ::testing::Test {
 protected:
  static int id_;
  static void test_thread_pool(int id) {
    id_ = std::max(id_, id);
  };
};

int sim_runner_small::id_ = 0;

TEST_F (sim_runner_small, thread_pool) {
  {
    scalesim::thr_pool thr_pool_(5);
    for (int id = 0; id < 32; ++id) {
      thr_pool_.post(boost::bind(&test_thread_pool, id));
    }
  }
  EXPECT_EQ(31, id_);
}

