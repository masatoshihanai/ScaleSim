/*
 * util_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "gtest/gtest.h"
#include "scalesim/util.hpp"

using namespace scalesim;

TEST (util_small, timestamp) {
  timestamp stmp0_0(0,0), stmp0_0_(0,0);
  EXPECT_TRUE(stmp0_0 == stmp0_0_);
  EXPECT_FALSE(stmp0_0 != stmp0_0_);

  timestamp stmp1_1(1,1);
  EXPECT_TRUE(stmp0_0 < stmp1_1);
  EXPECT_TRUE(stmp0_0 != stmp1_1);
  EXPECT_FALSE(stmp0_0 == stmp1_1);
  EXPECT_FALSE(stmp0_0 > stmp1_1);

  timestamp stmp1_0(1,0);
  EXPECT_TRUE(stmp0_0 < stmp1_0);
  EXPECT_TRUE(stmp0_0 != stmp1_0);
  EXPECT_FALSE(stmp0_0 > stmp1_0);
  EXPECT_FALSE(stmp0_0 == stmp1_0);

  timestamp stmp0_1(0,1);
  EXPECT_TRUE(stmp0_0 < stmp0_1);
  EXPECT_TRUE(stmp0_0 != stmp0_1);
  EXPECT_FALSE(stmp0_0 == stmp0_1);
  EXPECT_FALSE(stmp0_0 > stmp0_1);
}


