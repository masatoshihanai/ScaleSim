/*
 * io_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include <vector>
#include <boost/unordered_map.hpp>
#include "gtest/gtest.h"
#include "scalesim/util.hpp"

class input_small: public ::testing::Test {
 protected:
  const std::string GRAPH_FILE_10 = "traffic/ring/part/graph.part.10";
  const std::string GRAPH_FILE_9 = "traffic/ring/part/graph.part.9";
  const std::string TRAFFIC_MAP_RING = "traffic/ring/rd.sim.csv";
  const std::string TRAFFIC_TRIP_RING = "traffic/ring/trip.csv";
};

TEST_F(input_small, graph_partition_10) {
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr index(new boost::unordered_multimap<long, long>);
  scalesim::test_app::graph_read(GRAPH_FILE_10, result_partition, index);

  EXPECT_EQ(0, (*result_partition)[0]);
  EXPECT_EQ(1, (*result_partition)[1]);
  EXPECT_EQ(2, (*result_partition)[2]);
  EXPECT_EQ(3, (*result_partition)[3]);
  EXPECT_EQ(4, (*result_partition)[4]);
  EXPECT_EQ(5, (*result_partition)[5]);
  EXPECT_EQ(6, (*result_partition)[6]);
  EXPECT_EQ(7, (*result_partition)[7]);
  EXPECT_EQ(8, (*result_partition)[8]);
  EXPECT_EQ(9, (*result_partition)[9]);
}

TEST_F(input_small, graph_index_10) {
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr result_index(new boost::unordered_multimap<long, long>);
  scalesim::test_app::graph_read(GRAPH_FILE_10, result_partition, result_index);

  EXPECT_EQ(0, result_index->find(0)->second);
  EXPECT_EQ(1, result_index->find(1)->second);
  EXPECT_EQ(2, result_index->find(2)->second);
  EXPECT_EQ(3, result_index->find(3)->second);
  EXPECT_EQ(4, result_index->find(4)->second);
  EXPECT_EQ(5, result_index->find(5)->second);
  EXPECT_EQ(6, result_index->find(6)->second);
  EXPECT_EQ(7, result_index->find(7)->second);
  EXPECT_EQ(8, result_index->find(8)->second);
  EXPECT_EQ(9, result_index->find(9)->second);
}


TEST_F(input_small, graph_partition_9) {
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr index(new boost::unordered_multimap<long, long>);
  scalesim::test_app::graph_read(GRAPH_FILE_9, result_partition, index);

  EXPECT_EQ(0, (*result_partition)[0]);
  EXPECT_EQ(1, (*result_partition)[1]);
  EXPECT_EQ(2, (*result_partition)[2]);
  EXPECT_EQ(3, (*result_partition)[3]);
  EXPECT_EQ(4, (*result_partition)[4]);
  EXPECT_EQ(5, (*result_partition)[5]);
  EXPECT_EQ(6, (*result_partition)[6]);
  EXPECT_EQ(7, (*result_partition)[7]);
  EXPECT_EQ(8, (*result_partition)[8]);
  EXPECT_EQ(0, (*result_partition)[9]);
}

TEST_F(input_small, graph_index_9) {
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr result_index(new boost::unordered_multimap<long, long>);
  scalesim::test_app::graph_read(GRAPH_FILE_9, result_partition, result_index);

  EXPECT_EQ(0, result_index->find(0)->second);
  EXPECT_EQ(1, result_index->find(1)->second);
  EXPECT_EQ(2, result_index->find(2)->second);
  EXPECT_EQ(3, result_index->find(3)->second);
  EXPECT_EQ(4, result_index->find(4)->second);
  EXPECT_EQ(5, result_index->find(5)->second);
  EXPECT_EQ(6, result_index->find(6)->second);
  EXPECT_EQ(7, result_index->find(7)->second);
  EXPECT_EQ(8, result_index->find(8)->second);
  boost::unordered_multimap<long, long>::iterator it = result_index->find(0);
  it++;
  EXPECT_EQ(9, it->second);
}
