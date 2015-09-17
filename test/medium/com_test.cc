/*
 * com_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "gtest/gtest.h"
#include "scalesim/com.hpp"
#include "scalesim/logical_process.hpp"
#include "scalesim/simulation/sim_obj.hpp"

using namespace scalesim;

// TODO test of communicator
class com_test_medium: public ::testing::Test {

};

TEST_F (com_test_medium, send_receive) {
  EXPECT_TRUE(false);
}

TEST_F (com_test_medium, barrier) {
  EXPECT_TRUE(false);
}

TEST_F (com_test_medium, shuffle_events) {
  EXPECT_TRUE(false);
}

TEST_F (com_test_medium, reduce_sum) {
  EXPECT_TRUE(false);
}
