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
  phold simulation;
  simulation.init(); /* Init Random Latency */

  /* Test rank size = 1 */
  ev_vec<phold> init_events;
  simulation.init_events(init_events, 0, 1);

  EXPECT_EQ(NUM_INIT_MSG, init_events.size());
  for (auto itr = init_events.begin(); itr != init_events.end(); ++itr) {
    long id = (*itr)->id();
    /* Check Latency */
    EXPECT_EQ((*itr)->send_time(), LATENCY_TABLE[id % RAND_TABLE_SIZE]);
    EXPECT_EQ((*itr)->receive_time(), LATENCY_TABLE[id % RAND_TABLE_SIZE]);
  }

  /* Test multiple ranks (4 ranks) */
  ev_vec<phold> init_events_rank0of4;
  ev_vec<phold> init_events_rank1of4;
  ev_vec<phold> init_events_rank2of4;
  ev_vec<phold> init_events_rank3of4;

  simulation.init_events(init_events_rank0of4, 0, 4);
  simulation.init_events(init_events_rank1of4, 1, 4);
  simulation.init_events(init_events_rank2of4, 2, 4);
  simulation.init_events(init_events_rank3of4, 3, 4);

  EXPECT_EQ(NUM_INIT_MSG,
            init_events_rank0of4.size() + init_events_rank1of4.size()
            + init_events_rank2of4.size() + init_events_rank3of4.size());
  auto itr0 = init_events_rank0of4.begin();
  auto itr1 = init_events_rank1of4.begin();
  auto itr2 = init_events_rank2of4.begin();
  auto itr3 = init_events_rank3of4.begin();
  for (auto itr = init_events.begin(); itr != init_events.end(); ++itr) {
    long id = (*itr)->id();
    /* Check Latency */
    if (id % 4 == 0) {
      EXPECT_EQ((*itr)->id(), (*itr0)->id());
      EXPECT_EQ((*itr)->send_time(), (*itr0)->send_time());
      EXPECT_EQ((*itr)->receive_time(), (*itr0)->receive_time());
      ++itr0;
    } else if (id % 4 == 1) {
      EXPECT_EQ((*itr)->id(), (*itr1)->id());
      EXPECT_EQ((*itr)->send_time(), (*itr1)->send_time());
      EXPECT_EQ((*itr)->receive_time(), (*itr1)->receive_time());
      ++itr1;
    } else if (id % 4 == 2) {
      EXPECT_EQ((*itr)->id(), (*itr2)->id());
      EXPECT_EQ((*itr)->send_time(), (*itr2)->send_time());
      EXPECT_EQ((*itr)->receive_time(), (*itr2)->receive_time());
      ++itr2;
    } else if (id % 4 == 3) {
      EXPECT_EQ((*itr)->id(), (*itr3)->id());
      EXPECT_EQ((*itr)->send_time(), (*itr3)->send_time());
      EXPECT_EQ((*itr)->receive_time(), (*itr3)->receive_time());
      ++itr3;
    }
  }
}

TEST (phold, init_states) {
  const int rank_size = 4;
  phold simulation;
  auto parti_index = simulation.init_partition_index(rank_size);
  st_vec<phold> states_rank0;
  st_vec<phold> states_rank1;
  st_vec<phold> states_rank2;
  st_vec<phold> states_rank3;

  simulation.init_states_in_this_rank(states_rank0, 0, rank_size, parti_index.first);
  simulation.init_states_in_this_rank(states_rank1, 1, rank_size, parti_index.first);
  simulation.init_states_in_this_rank(states_rank2, 2, rank_size, parti_index.first);
  simulation.init_states_in_this_rank(states_rank3, 3, rank_size, parti_index.first);

  EXPECT_EQ(NUM_LP, states_rank0.size() + states_rank1.size() + states_rank2.size() + states_rank3.size());
  auto itr0 = states_rank0.begin();
  auto itr1 = states_rank1.begin();
  auto itr2 = states_rank2.begin();
  auto itr3 = states_rank3.begin();
  for (long i = 0; i < NUM_LP; ++i) {
    if (i % rank_size == 0) {
      EXPECT_EQ(i, (*itr0)->id());
      ++itr0;
    } else if (i % rank_size == 1) {
      EXPECT_EQ(i, (*itr1)->id());
      ++itr1;
    } else if (i % rank_size == 2) {
      EXPECT_EQ(i, (*itr2)->id());
      ++itr2;
    } else if (i % rank_size == 3) {
      EXPECT_EQ(i, (*itr3)->id());
      ++itr3;
    }
  }
}

TEST (phold, init_what_if) {
  EXPECT_TRUE(false);
}

TEST (phold, event_hander) {
  phold simulation;
  simulation.init(); /* Init Random Table */
  long ev_id = 999;
  long src_id = 0; long dst_id = 1;
  long rec_time = 10000; long send_time = 10090;
  int num_hops = 8;

  ev_ptr<phold> ev = boost::make_shared<event<phold> >(
      event<phold>(ev_id, src_id, dst_id, rec_time, send_time, num_hops));
  st_ptr<phold> st = boost::make_shared<state<phold> >(
      state<phold>(dst_id));

  auto ret = simulation.event_handler(ev, st);

  ev_ptr<phold> ret_ev = *(ret->first.begin());
  st_ptr<phold> ret_st = ret->second;
  EXPECT_EQ(ret_st->id(), dst_id);
  EXPECT_EQ(ret_ev->id(), ev_id);
  EXPECT_EQ(ret_ev->source(), dst_id);
  EXPECT_EQ(ret_ev->send_time(), rec_time);
  EXPECT_EQ(ret_ev->num_hops() - num_hops, 1);

  /* Latency of an event is LATENCY_TABLE[(event id) + (num_hops)] */
  int rand_table_id = (int)(ret_ev->id() + (long) ret_ev->num_hops()) % RAND_TABLE_SIZE;
  EXPECT_EQ(ret_ev->receive_time() - ret_ev->send_time(),
            LATENCY_TABLE[rand_table_id] + LOOK_AHEAD);

  /* Whether an event send remote or local is decided by REMOTE_COM_TABLE[(event id) + (num_hops)] */
  if (REMOTE_COM_TABLE[rand_table_id] == 1) {
    /* Case of remote (send different LP) */
    EXPECT_NE(ret_ev->destination(), dst_id);
  } else { /* REMOTE_COM_TABLE[rand_table_id] == 0 */
    /* Case of local (send same LP) */
    EXPECT_EQ(ret_ev->destination(), dst_id);
  }
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
