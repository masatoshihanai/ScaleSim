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
  scalesim::graph_reader reader;
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr index(new boost::unordered_multimap<long, long>);
  bool read_success = reader.read(GRAPH_FILE_10, result_partition, index);
  ASSERT_EQ(true, read_success);

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
  scalesim::graph_reader reader;
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr result_index(new boost::unordered_multimap<long, long>);
  bool read_success = reader.read(GRAPH_FILE_10, result_partition, result_index);
  ASSERT_EQ(true, read_success);

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
  scalesim::graph_reader reader;
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr index(new boost::unordered_multimap<long, long>);
  bool read_success = reader.read(GRAPH_FILE_9, result_partition, index);
  ASSERT_EQ(true, read_success);

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
  scalesim::graph_reader reader;
  parti_ptr result_partition(new std::vector<long>);
  parti_indx_ptr result_index(new boost::unordered_multimap<long, long>);
  bool read_success = reader.read(GRAPH_FILE_9, result_partition, result_index);
  ASSERT_EQ(true, read_success);

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

TEST_F(input_small, traffic_state) {
  scalesim::state_reader reader;
  boost::shared_ptr<std::vector<long> > partition(new std::vector<long>);
  for (int i = 0; i < 10; ++i) {
    partition->push_back(i%2);
  }
  bool read_success = reader.read(TRAFFIC_MAP_RING, 0, 2, partition);
  ASSERT_TRUE(read_success);
  std::vector<std::pair<long, std::string> > roads = reader.states();
  EXPECT_EQ("R,0,0,1,10,100,1", roads[0].second);
  EXPECT_EQ("R,2,2,3,10,100,1", roads[1].second);
  EXPECT_EQ("R,4,4,5,10,100,1", roads[2].second);
  EXPECT_EQ("R,6,6,7,10,100,1", roads[3].second);
  EXPECT_EQ("R,8,8,9,10,100,1", roads[4].second);
}

TEST_F(input_small, trip_only_first_vehicle) {
  scalesim::event_reader reader;
  bool read_success = reader.read(TRAFFIC_TRIP_RING, 0, 2);
  ASSERT_TRUE(read_success);
  std::vector<std::string> events = reader.events();
  EXPECT_EQ("TP,0,0,1, 0,1,2,3,4,5,6,7,8,9,0", events[0]);
}
