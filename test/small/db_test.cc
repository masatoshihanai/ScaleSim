/*
 * db_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include <boost/make_shared.hpp>
#include "gtest/gtest.h"

#include "scalesim/logical_process.hpp"
#include "scalesim/util.hpp"
#include "test/test_app.hpp"

using namespace scalesim;

class db_test_small: public ::testing::Test {
 protected:
  static void SetUpTestCase() {
    FLAGS_store_dir = "tmp_db_test_small";
    FLAGS_store_ip = "127.0.0.1";
    scalesim::store<test_app>::init(0);
  };

  static void TearDownTestCase() {
    scalesim::store<test_app>::finish();
    scalesim::store<test_app>::clear();
    FLAGS_store_dir = "";
    FLAGS_store_ip = "";
  };
};

TEST_F(db_test_small, put_get_event) {
  long lp_id_ = 0;
  /* initiate event */
  long id_ = 0; long send_t_ = 0; long rec_t_ = 10; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }
  ev_ptr<test_app> event_(
      new event<test_app>(id_, send_t_, rec_t_, dest_, path));

  /* put event */
  timestamp tmstmp_(event_->receive_time(), event_->id());
  scalesim::store<test_app>::ev_store()->put(tmstmp_, lp_id_, *event_);

  /* get event */
  ev_ptr<test_app> ret;
  scalesim::store<test_app>::ev_store()->get(tmstmp_, lp_id_, ret);

  EXPECT_EQ(event_->id(), ret->id());
}

TEST_F (db_test_small, zero_lookahead_put_get) {
  long lp_id_ = 0;
  /* initiate zero la events */
  long id0_ = 0; long send_t_ = 0; long rec_t_ = 10; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }
  ev_ptr<test_app> event0_(
      new event<test_app>(id0_, send_t_, rec_t_, dest_, path));

  long id1_ = 1;
  ev_ptr<test_app> event1_zero_la_(
      new event<test_app>(id1_, send_t_, rec_t_, dest_, path));

  /* put zero la event */
  timestamp tmstmp0_(event0_->receive_time(), event0_->id());
  scalesim::store<test_app>::ev_store()->put(tmstmp0_, lp_id_, *event0_);

  timestamp tmstmp1_(event1_zero_la_->receive_time(), event1_zero_la_->id());
  scalesim::store<test_app>::ev_store()->put(tmstmp1_, lp_id_, *event1_zero_la_);

  /* get event */
  ev_ptr<test_app> ret0_;
  scalesim::store<test_app>::ev_store()->get(tmstmp0_, lp_id_, ret0_);

  ev_ptr<test_app> ret1_;
  scalesim::store<test_app>::ev_store()->get(tmstmp1_, lp_id_, ret1_);

  EXPECT_EQ(event0_->id(), ret0_->id());
  EXPECT_EQ(event0_->receive_time(), ret0_->receive_time());

  EXPECT_EQ(event1_zero_la_->id(), ret1_->id());
  EXPECT_EQ(event1_zero_la_->receive_time(), ret1_->receive_time());
} /* zero_lookahead_put_get */

TEST_F(db_test_small, get_range) {
  long lp_id_ = 0;
  long send_t_ = 0; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }

  /* put events */
  for (int id_ = 0, rec_t_ = 0; rec_t_ < 1000; ++rec_t_, ++id_) {
    ev_ptr<test_app> event_(
        new event<test_app>(id_, send_t_, rec_t_, dest_, path));
    timestamp tmstmp_(rec_t_, id_);
    scalesim::store<test_app>::ev_store()->put(tmstmp_, lp_id_, *event_);
  }

  /* get range */
  timestamp tm_from_(200, 0);
  timestamp tm_to_ = timestamp::max();

  std::vector<ev_ptr<test_app> > ret_ev_;

  scalesim::store<test_app>::ev_store()
      ->get_range(tm_from_, tm_to_, lp_id_, ret_ev_);

  /* check events */
  EXPECT_EQ(ret_ev_.size(), 800);
  for (int i = 0; i < 800; ++i) {
    EXPECT_EQ(i + 200, ret_ev_[i]->id());
  }
}

TEST_F(db_test_small, get_range_multi_lps) {
  long lp_id_ = 0; long lp_id1_ = 1;
  long send_t_ = 0; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }

  /* put events lp = 0 */
  for (int id_ = 0, rec_t_ = 0; rec_t_ < 1000; ++rec_t_, ++id_) {
    ev_ptr<test_app> event_(
        new event<test_app>(id_, send_t_, rec_t_, dest_, path));
    timestamp tmstmp_(rec_t_, id_);
    scalesim::store<test_app>::ev_store()->put(tmstmp_, lp_id_, *event_);
  }

  /* put event lp = 1 */
  for (int id_ = 1000, rec_t_ = 0; rec_t_ < 1000; ++rec_t_, ++id_) {
    ev_ptr<test_app> event_(
        new event<test_app>(id_, send_t_, rec_t_, dest_, path));
    timestamp tmstmp_(rec_t_, id_);
    scalesim::store<test_app>::ev_store()->put(tmstmp_, lp_id1_, *event_);
  }

  /* get range */
  timestamp tm_from_(200, 0);
  timestamp tm_to_ = timestamp::max();

  std::vector<ev_ptr<test_app> > ret_ev_;

  scalesim::store<test_app>::ev_store()
      ->get_range(tm_from_, tm_to_, lp_id1_, ret_ev_);

  /* check events */
  EXPECT_EQ(ret_ev_.size(), 800);
  for (int i = 0; i < 800; ++i) {
    EXPECT_EQ(i + 1200, ret_ev_[i]->id());
  }
}

TEST_F (db_test_small, put_range) {
  long lp_id_ = 0;
  long send_t_ = 0; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }

  /* initiate event */
  std::vector<timestamp> keys_;
  std::vector<ev_ptr<test_app> > events_;
  for (long id_ = 0, rec_t_ = 0; rec_t_ < 1000; ++rec_t_, ++id_) {
    ev_ptr<test_app> event_(
        new event<test_app>(id_, send_t_, rec_t_, dest_, path));
    timestamp tmstmp_(rec_t_, id_);
    keys_.push_back(tmstmp_);
    events_.push_back(event_);
  }

  /* put range */
  scalesim::store<test_app>::ev_store()
      ->put_range(keys_, events_, lp_id_);

  /* check events */
  for (long i = 0; i < 1000; ++i) {
    ev_ptr<test_app> ret_;
    scalesim::store<test_app>::ev_store()->get(keys_[i], lp_id_, ret_);

    EXPECT_EQ(events_[i]->receive_time(), ret_->receive_time());
  }
}

TEST_F (db_test_small, get_set_state) {
  long lp_id_ = 999;
  /* initiate state */
  long state_time_ = 999;
  long state_id_ = 10;
  st_ptr<test_app> state_(new state<test_app>(state_id_));

  /* put state */
  timestamp tmstmp_(state_time_, state_->id());
  scalesim::store<test_app>::st_store()->put(tmstmp_, lp_id_, *state_);

  /* get state */
  st_ptr<test_app> ret;
  scalesim::store<test_app>::st_store()->get(tmstmp_, lp_id_, ret);

  EXPECT_EQ(state_id_, ret->id());
}

TEST_F (db_test_small, get_prev_state) {
  long lp_id_ = 100;
  /* initiate state */
  long state_time0_ = 0; long state_id0_ = 0;
  st_ptr<test_app> state0_(new state<test_app>(state_id0_));

  long state_time1_ = 1; long state_id1_ = 1;
  st_ptr<test_app> state1_(new state<test_app>(state_id1_));

  /* put state */
  timestamp tmstmp0_(state_time0_, state0_->id());
  scalesim::store<test_app>::st_store()->put(tmstmp0_, lp_id_, *state0_);

  timestamp tmstmp1_(state_time1_, state1_->id());
  scalesim::store<test_app>::st_store()->put(tmstmp1_, lp_id_, *state1_);

  /* get state */
  st_ptr<test_app> ret;
  timestamp ret_tmstmp;
  scalesim::store<test_app>::st_store()
      ->get_prev(tmstmp1_, lp_id_, ret, ret_tmstmp);
  EXPECT_EQ(state_id0_, ret->id());

  scalesim::store<test_app>::st_store()
      ->get_prev(tmstmp0_, lp_id_, ret, ret_tmstmp);
  EXPECT_EQ(state_id0_, ret->id());
}

TEST_F (db_test_small, get_prev_state_invalid_key) {
  long lp_id_ = 0;
  /* initiate state */
  long state_time1_ = 25; long state_id1_ = 1;
  st_ptr<test_app> state1_(new state<test_app>(state_id1_));

  long state_time99_ = 1000; long state_id2_ = 2;
  st_ptr<test_app> state2_(new state<test_app>(state_id2_));

  /* put state */
  timestamp tmstmp1_(state_time1_, state1_->id());
  scalesim::store<test_app>::st_store()->put(tmstmp1_, lp_id_, *state1_);

  timestamp tmstmp2_(state_time99_, state1_->id());
  scalesim::store<test_app>::st_store()->put(tmstmp2_, lp_id_, *state2_);

  /* get state by state_time = 50 */
  st_ptr<test_app> ret;
  timestamp ret_tmstmp;
  timestamp tmstmp50_(50, state1_->id());
  scalesim::store<test_app>::st_store()
      ->get_prev(tmstmp50_, lp_id_, ret, ret_tmstmp);
  EXPECT_EQ(state_id1_, ret->id());
  EXPECT_EQ(ret_tmstmp.time(), tmstmp1_.time());
}
