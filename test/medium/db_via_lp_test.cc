/*
 * db_via_lp_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include "gtest/gtest.h"

#include "scalesim/logical_process.hpp"
#include "scalesim/util.hpp"
#include "test/test_app.hpp"

using namespace scalesim;
using namespace std;

class db_via_lp_medium: public ::testing::Test {
 protected:
  static void SetUpTestCase() {
    FLAGS_diff_init = true;
    FLAGS_store_dir = "tmp_db_via_lp_medium";
    FLAGS_store_ip = "localhost";

    lp_mngr<test_app>::instance()->init_partition(partition_());
    lp_mngr<test_app>::instance()->init_lps(0, 1);
  };

  static void TearDownTestCase() {
    lp_mngr<test_app>::instance()->delete_lps(0);
    store<test_app>::clear();
    FLAGS_store_dir = "";
    FLAGS_store_ip = "";
    FLAGS_diff_init = false;
  };

  static pair<parti_ptr, parti_indx_ptr> partition_() {
    pair<parti_ptr, parti_indx_ptr>
      ret(parti_ptr(new std::vector<long>()),
          parti_indx_ptr(new boost::unordered_multimap<long, long>()));
    graph_reader reader;
    reader.read("traffic/ring/part/graph.part.9", ret.first, ret.second);
    return ret;
  };
};


TEST_F(db_via_lp_medium, put_get_event_via_logical_process) {
  long lp_id_ = 0;
  int num_event = 100;

  /* initiate events from 0 to 99 */
  long send_t_ = 0; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }
  vector<ev_ptr<test_app> > events_;
  for (long id_ = 0, rec_t_ = 0; id_ < num_event; ++id_, ++rec_t_) {
    events_.push_back(ev_ptr<test_app>(
        new event<test_app>(id_, send_t_, rec_t_, dest_, path)));
  }

  /* insert events from 0 to 99 */
  lp<test_app>* lp_;
  lp_mngr<test_app>::instance()->get_lp(lp_, lp_id_);
  for (long i = 0; i < num_event; ++i) {
    lp_->buffer(events_[i]);
  }
  vector<ev_ptr<test_app> > cancels_;
  lp_->flush_buf(cancels_);

  /* store event (via clear event function) */
  lp_->clear_old_ev(timestamp::max());

  /* check event */
  vector<timestamp> ret_tmstmp_;
  vector<ev_ptr<test_app> > ret_events_;
  store<test_app>::ev_store()->get_range(timestamp::zero(), timestamp::max(),
                                         lp_id_, ret_events_);
  for (int i = 0; i < num_event; ++i) {
    EXPECT_EQ(events_[i]->id(), ret_events_[i]->id());
  }

  /* get event from 50 to 99 */
  vector<ev_ptr<test_app> > ret_events_from_50_to_99_via_lp_;
  lp_->load_events(ret_events_from_50_to_99_via_lp_, timestamp(50, 0));
  /* check */
  EXPECT_EQ(50, ret_events_from_50_to_99_via_lp_.size());
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(50 + i, ret_events_from_50_to_99_via_lp_[i]->id());
  }

  /* get event from 0 to 49 */
  vector<ev_ptr<test_app> > ret_events_from_0_to_49_via_lp_;
  lp_->load_events(ret_events_from_0_to_49_via_lp_, timestamp::zero());
  /* check */
  EXPECT_EQ(50, ret_events_from_0_to_49_via_lp_.size());
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(i, ret_events_from_0_to_49_via_lp_[i]->id());
  }
}

TEST_F(db_via_lp_medium, put_get_cancel_via_logical_process) {
  //todo store cancel to db
  EXPECT_TRUE(false);
}

TEST_F(db_via_lp_medium, put_get_state_via_logical_process) {
  // todo stare state to db
  EXPECT_TRUE(false);
}

