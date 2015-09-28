/*
 * logical_process_test.cc
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#include <vector>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include "gtest/gtest.h"
#include "scalesim/logical_process.hpp"
#include "scalesim/simulation/sim_obj.hpp"
#include "scalesim/util.hpp"

#include "test/test_app.hpp"

using namespace std;
using namespace scalesim;

class lp_test_medium: public ::testing::Test {
 protected:
  /* for multi threading */
  class enqueue_runner {
   public:
    long id;
    lp<test_app>& lp_;
   public:
    enqueue_runner(long id, lp<test_app>& lp_): id(id), lp_(lp_){};
    void run() {
      long dest_ = 0; int path[100];
      for (int i = 0; i < 100; ++i) { path[i] = 1; }

      for (int i = 0; i < 1000; i++) {
        long id_ = i; long send_t_ = i; long rec_t_ = id*10000 + i;
        ev_ptr<test_app> event_(
            new event<test_app>(id_, send_t_, rec_t_, dest_, path));
        lp_.buffer(event_);
      }
    }
  };
};


TEST_F (lp_test_medium, insert_event) {
  /* initiate event */
  long id_ = 0; long send_t_ = 0; long rec_t_ = 10; long dest_ = 0;
  int path[100]; for (int i = 0; i < 100; ++i) { path[i] = 1; }
  ev_ptr<test_app> event_(new event<test_app>(id_, send_t_, rec_t_, dest_, path));

  lp<test_app> lp_;
  lp_.buffer(event_);
  vector<ev_ptr<test_app> > dump_can;
  lp_.flush_buf(dump_can);

  ev_ptr<test_app> actual;
  lp_.dequeue_event(actual);
  EXPECT_EQ(event_->receive_time(), actual->receive_time());

  ev_ptr<test_app> null_ptr;
  lp_.dequeue_event(null_ptr);
  EXPECT_FALSE(null_ptr);
}

TEST_F (lp_test_medium, dequeue_null_ptr) {
  lp<test_app> lp_;
  ev_ptr<test_app> ret;
  lp_.dequeue_event(ret);
  EXPECT_TRUE(!ret);
  /*
   * If there is no event to be able to process,
   * local_time() returns max().
   */
  EXPECT_EQ(lp_.local_time(), timestamp::max());
}

TEST_F (lp_test_medium, annihiate_inserted_event) {
  lp<test_app> lp_;
  /* initiate event */
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 11; long dest1_ = 0;
  int path1[100]; for (int i = 0; i < 100; ++i) { path1[i] = 1; }
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest1_, path1));

  /* insert events */
  lp_.buffer(event0_);
  lp_.buffer(event1_);
  vector<ev_ptr<test_app> > dump_can_;
  lp_.flush_buf(dump_can_);

  /* insert cancel event */
  ev_ptr<test_app> cancel_event0(new event<test_app>(*event0_));
  cancel_event0->change_cancel();
  lp_.buffer(cancel_event0);
  vector<ev_ptr<test_app> > dump_can;
  lp_.flush_buf(dump_can);

  ev_ptr<test_app> ret;
  lp_.dequeue_event(ret);
  EXPECT_EQ(1, ret->id());
}

TEST_F (lp_test_medium, annihiate_buffered_event) {
  lp<test_app> lp_;
  /* initiate events */
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 11; long dest1_ = 0;
  int path1[100]; for (int i = 0; i < 100; ++i) { path1[i] = 1; }
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest1_, path1));

  /* buffer events */
  lp_.buffer(event0_);
  lp_.buffer(event1_);

  /* insert cancel */
  ev_ptr<test_app> cancel_event0(new event<test_app>(*event0_));
  cancel_event0->change_cancel();
  lp_.buffer(cancel_event0);
  std::vector<ev_ptr<test_app> > dump_can;
  lp_.flush_buf(dump_can);

  ev_ptr<test_app> ret;
  lp_.dequeue_event(ret);
  EXPECT_EQ(1, ret->id());
}

TEST_F (lp_test_medium, buffer_double_events_single_cancel) {
  lp<test_app> lp_;
  /* initiate events */
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  /* initiate same event */
  ev_ptr<test_app> same_event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  /* initiate cancel */
  ev_ptr<test_app> cancel_event0_(new event<test_app>(*event0_));
  cancel_event0_->change_cancel();

  /* inserts 2 same events and 1 cancel */
  lp_.buffer(event0_);
  lp_.buffer(cancel_event0_);
  lp_.buffer(same_event0_);
  vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels);

  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  ev_ptr<test_app> ret_same;
  lp_.dequeue_event(ret_same);

  EXPECT_TRUE(ret0 != 0);
  EXPECT_EQ(0, ret0->id());
  EXPECT_FALSE(ret_same);
}

TEST_F (lp_test_medium, buffer_zero_lookahead_events) {
  lp<test_app> lp_;
  /* initiate event */
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  long id1_ = 1;
  ev_ptr<test_app> zero_la_event0_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));

  /* buffer zero look ahead events */
  lp_.buffer(event0_);
  lp_.buffer(zero_la_event0_);
  vector<ev_ptr<test_app> > dump;
  lp_.flush_buf(dump);

  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  ev_ptr<test_app> ret1;
  lp_.dequeue_event(ret1);

  EXPECT_TRUE(ret0 != 0);
  EXPECT_TRUE(ret1 != 0);
  EXPECT_EQ(0, ret0->id());
  EXPECT_EQ(10, ret0->receive_time());
  EXPECT_EQ(1, ret1->id());
  EXPECT_EQ(10, ret1->receive_time());
}

TEST_F (lp_test_medium, insert_zero_lookahead_events) {
  lp<test_app> lp_;
  /* initiate event */
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  long id1_ = 1;
  ev_ptr<test_app> zero_la_event0_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));

  /* insert event */
  lp_.buffer(event0_);
  std::vector<ev_ptr<test_app> > dump;
  lp_.flush_buf(dump);

  /* insert zero lookahead event */
  lp_.buffer(zero_la_event0_);
  lp_.flush_buf(dump);

  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  EXPECT_EQ(0, ret0->id());
  EXPECT_EQ(10, ret0->receive_time());

  ev_ptr<test_app> ret1;
  lp_.dequeue_event(ret1);
  EXPECT_EQ(1, ret1->id());
  EXPECT_EQ(10, ret1->receive_time());

  ev_ptr<test_app> ret2;
  lp_.dequeue_event(ret2);
  EXPECT_FALSE(ret2);
}

TEST_F (lp_test_medium, insert_zero_lookahead_events_cancels_check) {
  lp<test_app> lp_;
  /* initiate event */
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

  long id1_ = 1;
  ev_ptr<test_app> zero_la_event_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));

  /* insert event */
  std::vector<ev_ptr<test_app> > dump;
  lp_.buffer(event0_);
  lp_.flush_buf(dump);

  /* insert zero lookahead event */
  lp_.buffer(zero_la_event_);
  std::vector<ev_ptr<test_app> > cancel;
  lp_.flush_buf(cancel);

  EXPECT_EQ(0, cancel.size());
}

TEST_F (lp_test_medium, buffer_many_zero_lookahead_event) {
  lp<test_app> lp_;
  /* initiate events */
  std::vector<ev_ptr<test_app> > zero_lookahead_event;
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  for (int i = 0; i < 10; ++i) {
    zero_lookahead_event.push_back(
        ev_ptr<test_app>(new event<test_app>(i, send_t0_, rec_t0_, dest0_, path0)));
  }

  /* buffer events */
  for (int i = 0; i < 10; ++i) {
    lp_.buffer(zero_lookahead_event[i]);
  }

  /* insert buffer */
  vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels);

  for (int i = 0; i < 10; ++i) {
    ev_ptr<test_app> ret;
    lp_.dequeue_event(ret);
    EXPECT_EQ(i, ret->id());
    EXPECT_EQ(10, ret->receive_time());
  }
}

TEST_F (lp_test_medium, insert_many_zero_lookahead_event) {
  //TODO insert many zero la event
  EXPECT_TRUE(false);
}

TEST_F (lp_test_medium, set_zero_lookahead_cancel_event) {
  lp<test_app> lp_;
  /* initiate cancel */
  long id0_ = 0; long send_t10_ = 10; long rec_t0_ = 10; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> cancel0_(new event<test_app>(id0_, send_t10_, rec_t0_, dest0_, path0));
  cancel0_->change_cancel();

  long id1_ = 1;
  ev_ptr<test_app> zero_la_cancel_(new event<test_app>(id1_, send_t10_, rec_t0_, dest0_, path0));
  zero_la_cancel_->change_cancel();

  /* set zero lookahead cancels */
  lp_.set_cancel(cancel0_);
  lp_.set_cancel(zero_la_cancel_);

  /* insert event receive time = 5 (< 10) */
  long send_t0_ = 0; long rec_t5_ = 5;
  ev_ptr<test_app> event_(new event<test_app>(id0_, send_t0_, rec_t5_, dest0_, path0));
  lp_.buffer(event_);
  std::vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels);

  EXPECT_EQ(2, cancels.size());
  EXPECT_EQ(0, cancels[0]->id());
  EXPECT_EQ(1, cancels[1]->id());
  EXPECT_EQ(10, cancels[0]->send_time());
  EXPECT_EQ(10, cancels[1]->send_time());
}

TEST_F (lp_test_medium, cancel_one_of_buffered_zero_lookahead_events) {
  {
    lp<test_app> lp_;
    /* initiate event */
    long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
    int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
    ev_ptr<test_app> event_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

    long id1_ = 1;
    ev_ptr<test_app> zero_la_event_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    ev_ptr<test_app> zero_la_cancel_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    zero_la_cancel_->change_cancel();

    /* buffer event */
    lp_.buffer(event_);
    lp_.buffer(zero_la_event_);
    lp_.buffer(zero_la_cancel_);

    /* insert buffer */
    vector<ev_ptr<test_app> > cancels;
    lp_.flush_buf(cancels);
    ev_ptr<test_app> ret;
    lp_.dequeue_event(ret);
    ev_ptr<test_app> ret_null;
    lp_.dequeue_event(ret_null);
    EXPECT_TRUE(ret != 0);
    EXPECT_EQ(0, ret->id());
    EXPECT_FALSE(ret_null);
    EXPECT_EQ(0, cancels.size());
  }
  {
    lp<test_app> lp_;
    /* initiate event */
    long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
    int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
    ev_ptr<test_app> event_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

    long id1_ = 1;
    ev_ptr<test_app> zero_la_event_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    ev_ptr<test_app> zero_la_cancel_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    zero_la_cancel_->change_cancel();

    /* buffer event */
    lp_.buffer(zero_la_event_);
    lp_.buffer(event_);
    lp_.buffer(zero_la_cancel_);

    /* insert buffer */
    vector<ev_ptr<test_app> > cancels;
    lp_.flush_buf(cancels);
    ev_ptr<test_app> ret;
    lp_.dequeue_event(ret);
    ev_ptr<test_app> ret_null;
    lp_.dequeue_event(ret_null);

    EXPECT_TRUE(ret != 0);
    EXPECT_EQ(0, ret->id());
    EXPECT_FALSE(ret_null);
    EXPECT_EQ(0, cancels.size());
  }
}

TEST_F (lp_test_medium, cancel_one_of_inserted_zero_lookahead_events) {
  {
    lp<test_app> lp_;
    /* initiate event */
    long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
    int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
    ev_ptr<test_app> event_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

    long id1_ = 1;
    ev_ptr<test_app> zero_la_event_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    ev_ptr<test_app> zero_la_cancel_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    zero_la_cancel_->change_cancel();

    /* insert event */
    lp_.buffer(event_);
    lp_.buffer(zero_la_event_);
    vector<ev_ptr<test_app> > dump;
    lp_.flush_buf(dump);

    /* insert cancel */
    lp_.buffer(zero_la_cancel_);
    lp_.flush_buf(dump);
    ev_ptr<test_app> ret;
    lp_.dequeue_event(ret);
    ev_ptr<test_app> ret_null;
    lp_.dequeue_event(ret_null);
    EXPECT_TRUE(ret != 0);
    EXPECT_EQ(0, ret->id());
    EXPECT_FALSE(ret_null);
  }
  {
    lp<test_app> lp_;

    /* initiate event */
    long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
    int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
    ev_ptr<test_app> event_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));

    long id1_ = 1;
    ev_ptr<test_app> zero_la_event_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    ev_ptr<test_app> zero_la_cancel_(new event<test_app>(id1_, send_t0_, rec_t0_, dest0_, path0));
    zero_la_cancel_->change_cancel();

    lp_.buffer(zero_la_event_);
    lp_.buffer(event_);
    vector<ev_ptr<test_app> > dump;
    lp_.flush_buf(dump);

    lp_.buffer(zero_la_cancel_);
    lp_.flush_buf(dump);
    ev_ptr<test_app> ret;
    lp_.dequeue_event(ret);
    ev_ptr<test_app> ret_null;
    lp_.dequeue_event(ret_null);

    EXPECT_TRUE(ret != 0);
    EXPECT_EQ(0, ret->id());
    EXPECT_FALSE(ret_null);
  }
}

TEST_F (lp_test_medium, rollback_by_event) {
  /*
   *          v <= iterator
   * (step 0) [0] [1] [3]
   *             v
   * (step 1) [0] [1] [3]
   *                 v
   * (step 2) [0] [1] [3]
   *                     v
   * (step 3) [0] [1] [3]
   *                        v
   * (step 4) [0] [1] [2] [3] (insert event [2])
   *                 v
   * (step 5) [0] [1] [2] [3] (rollback to 2)
   *                     v
   * (step 6) [0] [1] [2] [3] (reprocess)
   *                        v
   * (step 7) [0] [1] [2] [3]
   */
  lp<test_app> lp_;
  /* initiate state and event */
  st_ptr<test_app> state_(new state<test_app>(1));
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1;
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest0_, path0));
  long id3_ = 3; long send_t3_ = 3; long rec_t3_ = 3;
  ev_ptr<test_app> event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest0_, path0));

  /* insert events 0, 1, 3 */
  lp_.buffer(event0_);
  lp_.buffer(event1_);
  lp_.buffer(event3_);
  vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels);

  /* dequeue and check events 0, 1, 3 */
  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  EXPECT_EQ(0, ret0->id());
  lp_.set_cancel(event0_);
  timestamp smstmp(event0_->send_time(), event0_->id());
  lp_.update_state(state_, smstmp);
  ev_ptr<test_app> ret1;
  lp_.dequeue_event(ret1);
  EXPECT_EQ(1, ret1->id());
  lp_.set_cancel(event1_);
  timestamp smstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(state_, smstmp1);
  ev_ptr<test_app> ret3;
  lp_.dequeue_event(ret3);
  EXPECT_EQ(3, ret3->id());
  lp_.set_cancel(event3_);
  timestamp smstmp3(event3_->send_time(), event3_->id());
  lp_.update_state(state_, smstmp3);

  /* event 2 for rollback  */
  long id2_ = 2; long send_t2_ = 2; long rec_t2_ = 2;
  ev_ptr<test_app> event2_rollback_(new event<test_app>(id2_, send_t2_, rec_t2_, dest0_, path0));

  /* buffer rollbacking event 2 */
  lp_.buffer(event2_rollback_);
  vector<ev_ptr<test_app> > cancel_events;

  /* insert event 2, rollback (to 2) happens */
  lp_.flush_buf(cancel_events);

  /* dequeue and check events */
  ev_ptr<test_app> ret2;
  lp_.dequeue_event(ret2);
  EXPECT_EQ(2, ret2->id());
  ev_ptr<test_app> ret3_;
  lp_.dequeue_event(ret3_);
  EXPECT_EQ(3, ret3_->id());

  EXPECT_EQ(1, cancel_events.size());
  EXPECT_EQ(3, cancel_events[0]->id());
  cancel_events.clear();
} /* rollback_by_event */

TEST_F (lp_test_medium, rollback_by_cancel_event) {
  /*
   *          v <= iterator
   * (step 0) [0] [1] [3]
   *             v
   * (step 1) [0] [1] [3]
   *                 v
   * (step 2) [0] [1] [3]
   *                     v
   * (step 3) [0] [1] [3]
   *                            v
   * (step 4) [0] [1] [can1] [3] (insert cancel 1)
   *                 v
   * (step 5) [0] [3] (cancel event 1)
   *             v
   * (step 6) [0] [3] (rollback to 1)
   *                 v
   * (step 7) [0] [3] (reprocess)
   */
  lp<test_app> lp_;
  /* initiate event */
  st_ptr<test_app> state_(new state<test_app>(1));
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1;
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest0_, path0));
  long id3_ = 3; long send_t3_ = 3; long rec_t3_ = 3;
  ev_ptr<test_app> event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest0_, path0));

  /* insert events 0, 1, 3 */
  lp_.buffer(event0_);
  lp_.buffer(event1_);
  lp_.buffer(event3_);
  vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels);

  /* dequeue and check event */
  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  EXPECT_EQ(0, ret0->id());
  lp_.set_cancel(event0_);
  timestamp tmstmp0(event0_->send_time(), event0_->id());
  lp_.update_state(state_, tmstmp0);
  ev_ptr<test_app> ret1;
  lp_.dequeue_event(ret1);
  EXPECT_EQ(1, ret1->id());
  lp_.set_cancel(event1_);
  timestamp tmstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(state_, tmstmp1);
  ev_ptr<test_app> ret3;
  lp_.dequeue_event(ret3);
  EXPECT_EQ(3, ret3->id());
  lp_.set_cancel(event3_);
  timestamp tmstmp3(event3_->send_time(), event3_->id());
  lp_.update_state(state_, tmstmp3);

  ev_ptr<test_app> event1_cancel_(new event<test_app>(*event1_));
  event1_cancel_->change_cancel();

  /* insert event1 */
  lp_.buffer(event1_cancel_);
  vector<ev_ptr<test_app> > cancel_events;
  lp_.flush_buf(cancel_events); /* cancel event1 and rollback to 1 */

  EXPECT_EQ(1, cancel_events[0]->id());
  ev_ptr<test_app> ret3_;
  lp_.dequeue_event(ret3_);
  EXPECT_EQ(3, ret3_->id());
  EXPECT_EQ(timestamp::max(), lp_.local_time());

  cancel_events.clear();
} /* rollback_by_cancel_event */

TEST_F (lp_test_medium, zero_lookahead_rollback) {
  /**         v <= iterator
   * (step 0) [0] [1-1] [3]
   *             v
   * (step 1) [0] [1-1] [3]
   *                   v
   * (step 2) [0] [1-1] [3]
   *                       v
   * (step 3) [0] [1-1] [3]
   *                            v
   * (step 4) [0] [1-1] [1-2] [3] (insert event [1-2]: id 2, rec time 1)
   *                   v
   * (step 5) [0] [1-1] [1-2] [3] (rollback to 1-2)
   *                        v
   * (step 6) [0] [1-1] [1-2] [3] (reprocess)
   *                            v
   * (step 7) [0] [1-1] [1-2] [3]
   */
  lp<test_app> lp_;
  /* initiate state and event */
  st_ptr<test_app> state_(new state<test_app>(1));
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1;
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest0_, path0));
  long id3_ = 3; long send_t3_ = 3; long rec_t3_ = 3;
  ev_ptr<test_app> event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest0_, path0));


  /* insert event 0, 1, 3 */
  lp_.buffer(event0_);
  lp_.buffer(event1_);
  lp_.buffer(event3_);
  vector<ev_ptr<test_app> > dump;
  lp_.flush_buf(dump);

  /* dequeue and check event */
  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  EXPECT_EQ(0, ret0->id());
  lp_.set_cancel(event0_);
  timestamp tmstmp0(event0_->send_time(), event0_->id());
  lp_.update_state(state_, tmstmp0);
  ev_ptr<test_app> ret1;
  lp_.dequeue_event(ret1);
  EXPECT_EQ(1, ret1->id());
  lp_.set_cancel(event1_);
  timestamp tmstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(state_, tmstmp1);
  ev_ptr<test_app> ret3;
  lp_.dequeue_event(ret3);
  EXPECT_EQ(3, ret3->id());
  lp_.set_cancel(event3_);
  timestamp tmstmp3(event3_->send_time(), event3_->id());
  lp_.update_state(state_, tmstmp3);

  long id2_ = 2;
  ev_ptr<test_app> event2_rollback_(new event<test_app>(id2_, send_t1_, rec_t1_, dest0_, path0));

  /* insert event 1-2 */
  lp_.buffer(event2_rollback_);
  vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels); /* rollback to 1-2 */

  ev_ptr<test_app> ret2;
  lp_.dequeue_event(ret2);
  EXPECT_EQ(2, ret2->id());
  ev_ptr<test_app> ret3_;
  lp_.dequeue_event(ret3_);
  EXPECT_EQ(3, ret3_->id());

  EXPECT_EQ(1, cancels.size());
  EXPECT_EQ(3, cancels[0]->id());
} /* zero_lookahead_rollback */

TEST_F (lp_test_medium, zero_lookahead_rollback_by_cancel) {
  /*
   *          v <= iterator
   * (step 0) [0] [1-1] [1-2] [3]
   *             v
   * (step 1) [0] [1-1] [1-2] [3]
   *                   v
   * (step 2) [0] [1-1] [1-2] [3]
   *                         v
   * (step 3) [0] [1-1] [1-2] [3]
   *                             v
   * (step 4) [0] [1-1] [1-2] [3] (insert cancel [1-2])
   *                   v
   * (step 5) [0] [1-1] [3]       (rollback to 1-2)
   *                       v
   * (step 6) [0] [1-1] [3]       (reprocess)
   *                       v
   * (step 7) [0] [1-1] [3]
   */
  lp<test_app> lp_;
  /* initiate state and event */
  st_ptr<test_app> state_(new state<test_app>(1));
  long id0_ = 0; long send_t0_ = 0; long rec_t0_ = 0; long dest0_ = 0;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event0_(new event<test_app>(id0_, send_t0_, rec_t0_, dest0_, path0));
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1;
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest0_, path0));
  long id2_ = 2;
  ev_ptr<test_app> event1_2_(new event<test_app>(id2_, send_t1_, rec_t1_, dest0_, path0));
  long id3_ = 3; long send_t3_ = 3; long rec_t3_ = 3;
  ev_ptr<test_app> event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest0_, path0));

  /* insert event 0, 1, 1-2, 3 */
  lp_.buffer(event0_);
  lp_.buffer(event1_);
  lp_.buffer(event1_2_);
  lp_.buffer(event3_);
  vector<ev_ptr<test_app> > dump;
  lp_.flush_buf(dump);

  /* dequeue and check event */
  ev_ptr<test_app> ret0;
  lp_.dequeue_event(ret0);
  EXPECT_EQ(0, ret0->id());
  lp_.set_cancel(event0_);
  timestamp tmstmp0(event0_->send_time(), event0_->id());
  lp_.update_state(state_, tmstmp0);
  ev_ptr<test_app> ret1;
  lp_.dequeue_event(ret1);
  EXPECT_EQ(1, ret1->id());
  lp_.set_cancel(event1_);
  timestamp tmstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(state_, tmstmp1);
  ev_ptr<test_app> ret2;
  lp_.dequeue_event(ret2);
  EXPECT_EQ(2, ret2->id());
  lp_.set_cancel(event1_2_);
  timestamp tmstmp1_2(event1_2_->send_time(), event1_2_->id());
  lp_.update_state(state_, tmstmp1_2);
  ev_ptr<test_app> ret3;
  lp_.dequeue_event(ret3);
  EXPECT_EQ(3, ret3->id());
  lp_.set_cancel(event3_);
  timestamp tmstmp3(event3_->send_time(), event3_->id());
  lp_.update_state(state_, tmstmp3);

  ev_ptr<test_app> cancel1_2_(new event<test_app>(*event1_2_));
  cancel1_2_->change_cancel();

  /* insert cancel 1-2 */
  lp_.buffer(cancel1_2_);
  vector<ev_ptr<test_app> > cancels;
  lp_.flush_buf(cancels); /* rollback to 1-2 */

  ev_ptr<test_app> ret3_;
  lp_.dequeue_event(ret3_);
  EXPECT_EQ(3, ret3_->id());

  EXPECT_EQ(2, cancels.size());
  EXPECT_EQ(2, cancels[0]->id());
  EXPECT_EQ(3, cancels[1]->id());
} /* zero_lookahead_rollback_by_cancel */

TEST_F (lp_test_medium, state_dequeue_update) {
  lp<test_app> lp_;
  st_ptr<test_app> state_(new state<test_app>(0));
  lp_.init_state(state_);
  st_ptr<test_app> ret;
  lp_.get_state(ret);
  EXPECT_EQ(0, ret->id_);

  st_ptr<test_app> new_state1(new state<test_app>(1));
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1; long dest1_ = 1;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest1_, path0));
  lp_.set_cancel(event1_);
  timestamp tmstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(new_state1, tmstmp1);

  st_ptr<test_app> ret1;
  lp_.get_state(ret1);
  EXPECT_EQ(1, ret1->id_);

  st_ptr<test_app> new_state2(new state<test_app>(2));
  long id2_ = 2; long send_t2_ = 2; long rec_t2_ = 2; long dest2_ = 2;
  ev_ptr<test_app> event2_(new event<test_app>(id2_, send_t2_, rec_t2_, dest2_, path0));
  lp_.set_cancel(event2_);
  timestamp tmstmp2(event2_->send_time(), event2_->id());
  lp_.update_state(new_state2, tmstmp2);
  st_ptr<test_app> ret2;
  lp_.get_state(ret2);
  EXPECT_EQ(2, ret2->id_);
}

TEST_F (lp_test_medium, state_rollback) {
  /*
   * Time    1,   2,   3,   4
   * State [1], [2],      [4]
   *                   ^ Insert Event to 3
   *                         => delete state [4]
   *
   * Time    1,   2,   3,   4
   * State [1], [2],
   *                   ^ local time = 3
   */
  lp<test_app> lp_;
  /* initiate states */
  st_ptr<test_app> state_(new state<test_app>(0));
  st_ptr<test_app> state1(new state<test_app>(1));
  st_ptr<test_app> state2(new state<test_app>(2));
  st_ptr<test_app> state4(new state<test_app>(4));
  /* initiate events 1, 2, 4 */
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1; long dest1_ = 1;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest1_, path0));
  long id2_ = 2; long send_t2_ = 2; long rec_t2_ = 4;
  ev_ptr<test_app> event2_(new event<test_app>(id2_, send_t2_, rec_t2_, dest1_, path0));
  long id4_ = 4; long send_t4_ = 4; long rec_t4_ = 4;
  ev_ptr<test_app> event4_(new event<test_app>(id4_, send_t4_, rec_t4_, dest1_, path0));

  std::vector<ev_ptr<test_app> > dummy;

  /* initiate state */
  lp_.init_state(state_);
  st_ptr<test_app> ret0;
  lp_.get_state(ret0);
  EXPECT_EQ(0, ret0->id_);

  /* insert state 1,2,4 */
  lp_.buffer(event1_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event1_);
  timestamp tmstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(state1, tmstmp1);
  ev_ptr<test_app> null_ptr;
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret1;
  lp_.get_state(ret1);
  EXPECT_EQ(1, ret1->id_);

  lp_.buffer(event2_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event2_);
  timestamp tmstmp2(event2_->send_time(), event2_->id());
  lp_.update_state(state2, tmstmp2);
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret2;
  lp_.get_state(ret2);
  EXPECT_EQ(2,ret2->id_);

  lp_.buffer(event4_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event4_);
  timestamp tmstmp4(event4_->send_time(), event4_->id());
  lp_.update_state(state4, tmstmp4);
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret4;
  lp_.get_state(ret4);
  EXPECT_EQ(4, ret4->id_);

  /* insert message to 3 */
  long id3_ = 3; long send_t3_ = 3; long rec_t3_ = 3;
  ev_ptr<test_app> event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest1_, path0));
  lp_.buffer(event3_);
  lp_.flush_buf(dummy);
  st_ptr<test_app> ret;
  lp_.get_state(ret);
  EXPECT_EQ(2, ret->id_);
}

TEST_F (lp_test_medium, state_rollback_by_cancel) {
  /*
   * Time    1,   2,   3,   4
   * State [1], [2], [3], [4]
   *                  ^ cancel message of 3
   *                        => delete state 3, 4
   *
   * Time    1,   2,   3,   4
   * State [1], [2],
   *                  ^ local time = 3
   */
  lp<test_app> lp_;
  st_ptr<test_app> state_(new state<test_app>(0));
  st_ptr<test_app> state1(new state<test_app>(1));
  st_ptr<test_app> state2(new state<test_app>(2));
  st_ptr<test_app> state3(new state<test_app>(3));
  st_ptr<test_app> state4(new state<test_app>(4));

  /* initiate events 1, 2, 3, 4 */
  long id1_ = 1; long send_t1_ = 1; long rec_t1_ = 1; long dest1_ = 1;
  int path0[100]; for (int i = 0; i < 100; ++i) { path0[i] = 1; }
  ev_ptr<test_app> event1_(new event<test_app>(id1_, send_t1_, rec_t1_, dest1_, path0));
  long id2_ = 2; long send_t2_ = 2; long rec_t2_ = 2;
  ev_ptr<test_app> event2_(new event<test_app>(id2_, send_t2_, rec_t2_, dest1_, path0));
  long id3_ = 3; long send_t3_ = 3; long rec_t3_ = 3;
  ev_ptr<test_app> event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest1_, path0));
  long id4_ = 4; long send_t4_ = 4; long rec_t4_ = 4;
  ev_ptr<test_app> event4_(new event<test_app>(id4_, send_t4_, rec_t4_, dest1_, path0));

  std::vector<ev_ptr<test_app> > dummy;

  /* initiate state */
  lp_.init_state(state_);
  st_ptr<test_app> ret0;
  lp_.get_state(ret0);
  EXPECT_EQ(0, ret0->id_);

  /* insert state 1,2,3,4 */
  lp_.buffer(event1_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event1_);
  timestamp tmstmp1(event1_->send_time(), event1_->id());
  lp_.update_state(state1, tmstmp1);
  ev_ptr<test_app> null_ptr;
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret1;
  lp_.get_state(ret1);
  EXPECT_EQ(1, ret1->id_);

  lp_.buffer(event2_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event2_);
  timestamp tmstmp2(event2_->send_time(), event2_->id());
  lp_.update_state(state2, tmstmp2);
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret2;
  lp_.get_state(ret2);
  EXPECT_EQ(2,ret2->id_);

  lp_.buffer(event3_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event3_);
  timestamp tmstmp3(event3_->send_time(), event3_->id());
  lp_.update_state(state3, tmstmp3);
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret3;
  lp_.get_state(ret3);
  EXPECT_EQ(3,ret3->id_);

  lp_.buffer(event4_);
  lp_.flush_buf(dummy);
  lp_.set_cancel(event4_);
  timestamp tmstmp4(event4_->send_time(), event4_->id());
  lp_.update_state(state4, tmstmp4);
  lp_.dequeue_event(null_ptr);
  st_ptr<test_app> ret4;
  lp_.get_state(ret4);
  EXPECT_EQ(4, ret4->id_);

  /* insert cancel message3 */
  ev_ptr<test_app> can_event3_(new event<test_app>(id3_, send_t3_, rec_t3_, dest1_, path0));
  can_event3_->change_cancel();
  lp_.buffer(can_event3_);
  lp_.flush_buf(dummy);
  st_ptr<test_app> ret;
  lp_.get_state(ret);
  EXPECT_EQ(2, ret->id_);
}

TEST_F (lp_test_medium, zero_lookahead_state) {
  // TODO zero_lookahead_state
  EXPECT_TRUE(false);
}

TEST_F (lp_test_medium, zero_lookahead_state_rollback) {
  //TODO zero_lookahead_state_rollback
  EXPECT_TRUE(false);
}

TEST_F (lp_test_medium, enqueue_1000_events_per_thr_from_100_threads) {
  /* create thread */
  boost::thread_group thr_gr;
  lp<test_app> lp_;
  const int thread_num = 100;
  vector<enqueue_runner*> runners;
  for (int i = 0; i < thread_num; i++) {
    runners.push_back(new enqueue_runner(i, lp_));
  }

  /* enqueue 1000 events in each thread */
  for (int i = 0; i < thread_num; i++) {
    thr_gr.create_thread(boost::bind(&enqueue_runner::run, runners[i]));
  }
  thr_gr.join_all();

  /* insert buffered events*/
  vector<ev_ptr<test_app> > dump;
  lp_.flush_buf(dump);

  /* dequeue events */
  ev_ptr<test_app> ret;
  for (long i = 0; i < thread_num; i++) {
    for (long j = 0; j < 1000; j++) {
      long expected = i*10000 + j;
      lp_.dequeue_event(ret);
      EXPECT_EQ(expected, ret->receive_time());
    }
  }

  for (long i = 0; i < thread_num; i++) {
    delete runners[i];
  }
  runners.clear();
}

/* lp_mngr_test */

class lp_mngr_test_mediuml: public ::testing::Test {
 protected:
  static pair<parti_ptr, parti_indx_ptr> partition_() {
    pair<parti_ptr, parti_indx_ptr>
        ret(parti_ptr(new std::vector<long>()),
            parti_indx_ptr(new boost::unordered_multimap<long, long>()));
    scalesim::test_app::graph_read("traffic/ring/part/graph.part.9",
                                   ret.first,
                                   ret.second);
    return ret;
  };
};

TEST_F (lp_mngr_test_mediuml, get_lp) {
  scalesim::lp_mngr<test_app> lp_manager_;
  lp_manager_.init_partition(partition_());
  lp_manager_.init_lps(0, 1);

  lp<test_app>* lp0;
  lp_manager_.get_lp(lp0, 0);
  EXPECT_EQ(0, lp0->id());

  lp<test_app>* lp9;
  lp_manager_.get_lp(lp9, 9);
  EXPECT_EQ(9, lp9->id());

  lp_manager_.delete_lps(0);
}
