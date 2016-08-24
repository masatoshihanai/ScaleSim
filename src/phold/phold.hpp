/*
 * phold.hpp
 *
 *  Copyright (c) 2016 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#ifndef PHOLD_PHOLD_HPP_
#define PHOLD_PHOLD_HPP_

#include <random>
#include <string>
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include "scalesim/simulation.hpp"
#include "scalesim/util.hpp"

/**
 * Benchmark program based on phold
 *
 * References
 *   "Warp Speed: Executing Time Warp on 1,966,080 Cores" (PADS '14)
 *     - http://dl.acm.org/citation.cfm?id=2486134
 *   "On Deciding Between Conservative and Optimistic Approaches
 *    on Massively Parallel Platforms" (WSC '10)
 *     - http://dl.acm.org/citation.cfm?id=2433588
 *   "Scalable Time Warp on Blue Gene Supercomputers" (PADS '09)
 *     - http://dl.acm.org/citation.cfm?id=1577971
 */

/* Parameter of PHOLD */
static long NUM_LP = 100;
static long NUM_INIT_MSG = 1600;
static double REMOTE_COM_RATIO = 0.1;
static double LAMBDA = 1.0;
static long RANDOM_SEED = 1;

/* Parameter of what-if */
static long NUM_WHAT_IF = 1;
static std::vector<long> TIME_OF_WHATIF;
static std::vector<long> LP_OF_WHATIF;

/* Parameter of System */
static long GSYNC_INTERVAL = 10;
static long LP_INTERVAL = 4;
static int GTW_CUT_INTERVAL = 50;

static const int EFFECTIVE_DECIMAL = 100000;
static const long LOOK_AHEAD = 0.1 * EFFECTIVE_DECIMAL;
static const int RAND_TABLE_SIZE = 10000000;
static long LATENCY_TABLE[RAND_TABLE_SIZE];
/* If value is 1, send to remote. Else if value is 0, send to local */
static int REMOTE_COM_TABLE[RAND_TABLE_SIZE];

class phold: public scalesim::application {
 public:
  class Event: public scalesim::sim_event {
   friend class phold;
   private:
    long id_; long src_id_; long dst_id_;
    long receive_time_; long send_time_; int num_hops_;
    mutable scalesim::sim_event_base base_;
   public:
    Event(): id_(-1), src_id_(-1), dst_id_(-1),
             receive_time_(-1), send_time_(-1), num_hops_(0) {};
    virtual ~Event(){};
    Event(long event_id, long src_id, long dst_id,
          long receive_time, long send_time, int num_hops):
            id_(event_id), src_id_(src_id), dst_id_(dst_id),
            receive_time_(receive_time), send_time_(send_time),
            num_hops_(num_hops) {};
    Event(const Event& event) {
      id_ = event.id_; src_id_ = event.src_id_; dst_id_ = event.dst_id_;
      receive_time_ = event.receive_time_; send_time_ = event.send_time_;
      num_hops_ = event.num_hops_;
      base_ = event.base_;
    };

   public:
    scalesim::sim_event_base* base() const { return &base_; };
    long id() const { return id_; };
    long source() const { return src_id_; };
    long destination() const { return dst_id_; };
    bool end() const { return true; };
    long receive_time() const { return receive_time_; };
    long send_time() const { return send_time_; };
    int size() const { return sizeof(*this); };
    int num_hops() const { return num_hops_; };

    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & id_;
      ar & src_id_;
      ar & dst_id_;
      ar & receive_time_;
      ar & send_time_;
      ar & num_hops_;
      ar & base_;
    }
  }; /* class event */

  class State : public scalesim::sim_state {
    friend class phold;
   private:
    long id_;
   public:
    State(): id_(-1) {};
    virtual ~State() {};
    State(long id): id_(id){};
    long id() const { return id_; }
    int size() const { return sizeof(*this); }
    void out_put() const {
      std::cout << "state id: " << id_ << std::endl;
    };
    friend class boost::serialization::access;
   private:
    template<class Archive>
    void serialize(Archive& ar, unsigned int version) {
      ar & id_;
    }
  }; /* class State */

 public:
  /* System configuration */
  static long gsync_interval() { return GSYNC_INTERVAL; };
  static long switch_lp_interval() { return LP_INTERVAL; };
  static int  global_cut_interval() { return GTW_CUT_INTERVAL; };
  static int num_thr() { return boost::thread::physical_concurrency(); };

  /* Application configuration */
  static long finish_time() { return 30 * EFFECTIVE_DECIMAL; }

  /*
   * Initiation function for application.
   * It is invoked before all initiation functions.
   */
  void init() {
    /* Make random numbers table for deciding latency */
    int ex_seed = RANDOM_SEED;
    std::default_random_engine ex_generator(ex_seed);
    std::exponential_distribution<double> ex_distribution(LAMBDA);
    for (int i = 0; i < RAND_TABLE_SIZE; ++i) {
      LATENCY_TABLE[i] = (long) (ex_distribution(ex_generator) * EFFECTIVE_DECIMAL);
    }

    /* Make random numbers table for deciding next logical process */
    int uni_seed = RANDOM_SEED;
    std::default_random_engine uni_generator(uni_seed);
    std::uniform_real_distribution<double> uni_distribution(0.0, 1.0);
    for (int i = 0; i < RAND_TABLE_SIZE; ++i) {
      if (uni_distribution(uni_generator) < REMOTE_COM_RATIO) {
        REMOTE_COM_TABLE[i] = 1; /* case of remote */
      } else {
        REMOTE_COM_TABLE[i] = 0; /* case of local */
      }
    }
  };

  /*
   * Initiation function for partition and index.
   * Partition format:
   *   - type: boost::shared_ptr<std::std::vector<long> >
   *   - value: Nth value represents a rank number in ID=N
   * Index format:
   *   - type: boost::shared_ptr<boost::unordered_multimap<long, long> >
   *   - key: rank number
   *   - value: IDs in this rank
   */
  std::pair<parti_ptr, parti_indx_ptr> init_partition_index(int rank_size) {
    auto partition_ = boost::make_shared<std::vector<long> > (std::vector<long>());
    auto index_ = boost::make_shared<boost::unordered_multimap<long, long> >(
        boost::unordered_multimap<long, long>());

    /* Naive round robin partitioning based on remainder. */
    for (long i = 0; i < NUM_LP; ++i) {
      partition_->push_back(i % rank_size);
      auto kv = std::pair<int, long>(i % rank_size, i);
      index_->insert(kv);
    }

    return std::pair<parti_ptr, parti_indx_ptr>(partition_, index_);
  };

  /*
   * Initiation function for events.
   * Initiated events are shuffled to each starting point after this function.
   * Thus it is ok to just read in whatever way.
   * For example, use modulo operator based on rank_id and rank_size.
   */
  void init_events(ev_vec<phold>& ret,
                   const int rank,
                   const int rank_size) {
    for (long id = 0; id < NUM_INIT_MSG; ++id) {
      if (id % rank_size == rank) {
        ret.push_back(boost::make_shared<event<phold> > (
            event<phold>(id, id % NUM_LP, id % NUM_LP,
                         LATENCY_TABLE[id % RAND_TABLE_SIZE],
                         LATENCY_TABLE[id % RAND_TABLE_SIZE],
                         0)));
      }
    }
  };

  /*
   * Initiation function for states.
   * Initiated states are NOT shuffled after this function.
   * Thus you have to initiate only states in this rank, based on partition.
   */
  void init_states_in_this_rank(st_vec<phold>& new_state,
                                const int rank,
                                const int rank_size,
                                parti_ptr partition) {
    for (long id = 0; id < NUM_LP; ++id) {
      if ((*partition)[id] == rank) {
        new_state.push_back(boost::make_shared<state<phold> > (state<phold>(id)));
      }
    }
  };

  /*
   * Initiation function for what_if events.
   * Initiated events are shuffled to each starting point after this function.
   * Thus it is ok to just read in whatever way.
   */
  void init_what_if(
      std::vector<boost::shared_ptr<const scalesim::what_if<phold> > >& ret,
      const int rank,
      const int rank_size) {
    /* Generate what-if scenario */
    for (long i = 0; i < NUM_WHAT_IF; ++i) {
      LP_OF_WHATIF.push_back(i); /* Just select LP0, LP1, LP2,,, */
      TIME_OF_WHATIF.push_back(1); /* Every what-ifs is from 1 * EFFECTIVE_DECIMAL */
    }

    /* Init what-if scenario */
    for (long i = 0; i < NUM_WHAT_IF; ++i) {
      long lp_id = LP_OF_WHATIF[i];
      long time = TIME_OF_WHATIF[i];
      event<phold> add_event(NUM_INIT_MSG + i, lp_id, lp_id, time, time, 0);
      ret.push_back(boost::make_shared<scalesim::what_if<phold> >(
          scalesim::what_if<phold>(lp_id, time, add_event)));
    }
  };

  /*
   * Event handling function.
   * The arguments (receive_event, state) are previous value in the simulation.
   * The return value (optional<pair<ev_pst, st_ptr> >) should include
   * new event and state based on the arguments and your simulation models.
   *
   * If there are no new event and state generated, return empty option.
   */
  boost::optional<std::pair<ev_vec<phold>, st_ptr<phold> > >
  event_handler(ev_ptr<phold> receive_event, st_ptr<phold> state) {
    ev_vec<phold> new_events;
    long ev_id = receive_event->id();
    long src_id = receive_event->destination();
    long send_time = receive_event->receive_time();
    int num_hops = 1 + receive_event->num_hops();

    int rand_table_id = (int) (ev_id + (long) num_hops) % RAND_TABLE_SIZE;
    long receive_time = send_time + LATENCY_TABLE[rand_table_id] + LOOK_AHEAD;
    long dst_id = -1;
    if (REMOTE_COM_TABLE[rand_table_id] == 1) {
      /* Case of remote (send to different LP) */
      dst_id = LATENCY_TABLE[rand_table_id] % NUM_LP;
    } else { /* REMOTE_COM_TABLE[rand_table_id] == 0 */
      /* Case of local (send to this LP) */
      dst_id = receive_event->destination();
    }

    new_events.push_back(
        boost::make_shared<event<phold> >(
            event<phold>(ev_id, src_id, dst_id, receive_time, send_time, num_hops)));

    return boost::optional<std::pair<ev_vec<phold>, st_ptr<phold> > > (
        std::pair<ev_vec<phold>, st_ptr<phold> >(new_events, state));
  };
};

#endif /* PHOLD_PHOLD_HPP_ */
