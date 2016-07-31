/*
 * main.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "leveldb/db.h"
#include "leveldb/options.h"

/* test files */
#include "medium/com_test.cc"
#include "medium/db_via_lp_test.cc"
#include "medium/gvt_test.cc"
#include "medium/logical_process_test.cc"
#include "small/db_test.cc"
#include "small/io_test.cc"
#include "small/thread_test.cc"
#include "small/util_test.cc"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(*argv);
  google::InstallFailureSignalHandler();

  return RUN_ALL_TESTS();
}
