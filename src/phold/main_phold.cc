/*
 * main_phold.cc
 *
 *  Copyright (c) 2016 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#include <random>
#include "scalesim/simulation.hpp"
#include "scalesim/util.hpp"
#include "phold.hpp"

using namespace std;
static const int EFFECTIVE_DECIMAL = 100000;

static long NUM_LP = 100;
static long NUM_INIT_MSG = 1600;
static double REMOTE_COM_RATIO = 0.1;
static long LOOK_AHEAD = 0.1 * EFFECTIVE_DECIMAL;
static long FINISH_TIME = 100000 * EFFECTIVE_DECIMAL;

static const int EX_RAND_TABLE_SIZE = 10000;
static long LATENCY_TABLE[EX_RAND_TABLE_SIZE];
static const int UNI_RAND_TABLE_SIZE = 1000;
static int REMOTE_COM_TABLE[UNI_RAND_TABLE_SIZE];

long phold::finish_time() {
  return FINISH_TIME;
}

void phold::init() {
  /* Make random numbers table for deciding latency */
  int ex_seed = 1;
  std::default_random_engine ex_generator(ex_seed);
  double lamda = 3.5;
  std::exponential_distribution<double> ex_distribution(lamda);
  for (int i = 0; i < EX_RAND_TABLE_SIZE; ++i) {
    LATENCY_TABLE[i] = (long) (ex_distribution(ex_generator) * EFFECTIVE_DECIMAL);
  }

  /* Make random numbers table for deciding next logical process */
  int uni_seed = 1;
  std::default_random_engine uni_generator(uni_seed);
  std::uniform_real_distribution<double> uni_distribution(0.0, 1.0);
  for (int i = 0; i < UNI_RAND_TABLE_SIZE; ++i) {
    REMOTE_COM_TABLE[i] = (int) (uni_distribution(uni_generator) * EFFECTIVE_DECIMAL);
  }
}

pair<parti_ptr, parti_indx_ptr> phold::init_partition_index(int rank_size) {
  auto partition_ = boost::make_shared<vector<long> > (vector<long>());
  auto index_ = boost::make_shared<boost::unordered_multimap<long, long> >(
                  boost::unordered_multimap<long, long>());

  /* Naive round robin partitioning based on remainder. */
  for (long i = 0; i < NUM_LP; ++i) {
    partition_->push_back(i % rank_size);
    auto kv = pair<int, long>(i % rank_size, i);
    index_->insert(kv);
  }

  return pair<parti_ptr, parti_indx_ptr>(partition_, index_);
};

void phold::init_events(ev_vec<phold>& ret,
                        const int rank,
                        const int rank_size) {
  for (long id = 0; id < NUM_INIT_MSG; ++id) {
    if (id % rank_size == rank) {
      ret.push_back(boost::make_shared<event<phold> > (
          event<phold>(id, id % NUM_LP, id % NUM_LP,
                       LATENCY_TABLE[id % EX_RAND_TABLE_SIZE],
                       LATENCY_TABLE[id % EX_RAND_TABLE_SIZE],
                       0)));
    }
  }
};

void phold::init_states_in_this_rank(st_vec<phold>& new_state,
                                     const int rank,
                                     const int rank_size,
                                     parti_ptr partition) {
  for (long id = 0; id < NUM_LP; ++id) {
    if ((*partition)[id] == rank) {
      new_state.push_back(boost::make_shared<state<phold> > (state<phold>(id)));
    }
  }
};

void phold::init_what_if(
         vector<boost::shared_ptr<const scalesim::what_if<phold> > >& ret,
         const int rank,
         const int rank_size) {
  // TODO
};


boost::optional<pair<vector<ev_ptr<phold> >, st_ptr<phold> > >
phold::event_handler(ev_ptr<phold> receive_event,
                     st_ptr<phold> state) {
  // TODO
};

int main(int argc, char* argv[]) {
  scalesim::runner<phold>::init_main(&argc, &argv);
  scalesim::runner<phold>::run();
  return 0;
};
