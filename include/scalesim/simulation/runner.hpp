/*
 * runner.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_SIMULATION_SIM_RUNNER_HPP_
#define SCALESIM_SIMULATION_SIM_RUNNER_HPP_

#include <stdlib.h>
#include <sys/time.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "scalesim/com.hpp"
#include "scalesim/logical_process.hpp"
#include "scalesim/simulation.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template <class App>
class runner {
 private:
  App app_;
 private:
  runner(){};
  virtual ~runner(){};
  runner(const runner&);
  void operator=(const runner&);

 private:
  int simulation();

  void init();
  void init_repeat();
  void finish();

  void run_lp_aggr(lp<App>* lp_, thr_pool* thr_pool);
  void clear_old_ev_st(lp<App>* lp, thr_pool* pool, const timestamp& time);
  void store_old_ev_st(lp<App>* lp, thr_pool* pool, const timestamp& time);
  void output(const timestamp& from, const timestamp& to);

 public:
  static void init_main(int* argc, char** argv[]);
  static void run();
};

template <class App>
int runner<App>::simulation() {
  if (FLAGS_diff_repeat) {
    init_repeat();
  } else { /* initial what-if simulation or normal simulation */
    init();
  }

  thr_pool pool(App::thr_pool_size());

  LOG_IF(INFO, event_com<App>::instance()->rank() == 0) << "Start Simulation";
  stopwatch::instance("Simulation")->start();
  while (true) {
    /* processing events with threads */
    stopwatch::instance("EventProcessing")->start();
    for (auto it = lp_mngr<App>::instance()->get_lps().begin();
        it != lp_mngr<App>::instance()->get_lps().end(); ++it) {
      pool.post(boost::bind(&runner::run_lp_aggr, this, it->second, &pool));
      // pool.enqueue(boost::bind(&sim_runner::run_lp_lazy, this, it->second));
    }
    pool.wait_all();

    stopwatch::instance("EventProcessing")->stop();

    /* compute local minimum time */
    timestamp local_min_time_ = timestamp::max();
    for (auto it = lp_mngr<App>::instance()->get_lps().begin();
        it != lp_mngr<App>::instance()->get_lps().end(); ++it) {
      local_min_time_ = std::min(local_min_time_, it->second->local_time());
    }
    gvt_com<App>::instance()->update_local_min(local_min_time_);
    gvt_com<App>::instance()->increment_gsync_interval();

    if (gvt_com<App>::instance()->check_updated()) {
      /* output result */
      output(gvt_com<App>::instance()->gtime_prev(), gvt_com<App>::instance()->gtime());

      /* clear event and state */
      for (auto it = lp_mngr<App>::instance()->get_lps().begin();
           it != lp_mngr<App>::instance()->get_lps().end(); ++it) {
        pool.post(boost::bind(&runner::clear_old_ev_st,
                              this,
                              it->second,
                              &pool,
                              gvt_com<App>::instance()->gtime()));
      }
      pool.wait_all();
    }

    /* finish */
    if (gvt_com<App>::instance()->gtime().time() >= App::finish_time()) {
      break;
    }
  } /* while loop */

  stopwatch::instance("Simulation")->stop();

  finish();
  return 0;
};

template <class App>
void runner<App>::init() {
   stopwatch::instance("InitApp")->start();

   /* initiate application */
   app_.init();

   /* initiate communication manager */
   event_com<App>::instance()->init(lp_mngr<App>::instance());

   LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
       << "Initiate application";
   LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
       << "Initiate lp manager";
   LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
       << "Initiate com  manager";

   /* initiate logical processes */
   LOG_IF(INFO, event_com<App>::instance()->rank() == 0) << "Initiate lps";
   lp_mngr<App>::instance()->init_partition(app_.init_partition_index());
   lp_mngr<App>::instance()->init_lps(event_com<App>::instance()->rank(),
                                      event_com<App>::instance()->rank_size());

   stopwatch::instance("InitApp")->stop();

   /* initiate state */
   stopwatch::instance("InitState")->start();
   LOG_IF(INFO, event_com<App>::instance()->rank() == 0) << "Initiate states";
   st_vec<App> states;
   app_.init_states_in_this_rank(states,
                                 event_com<App>::instance()->rank(),
                                 event_com<App>::instance()->rank_size(),
                                 lp_mngr<App>::instance()->partition());
   for (auto it = states.begin(); it != states.end(); ++it) {
     lp<App>* lp_;
     lp_mngr<App>::instance()->get_lp(lp_, (*it)->id());
     lp_->init_state(*it);
   }
   bool wait = true;
   long sum_states = 0;
   event_com<App>::instance()->reduce_sum(wait, sum_states, states.size());
   while (wait);

   long byte_state = sizeof(state<App>);
   stopwatch::instance("InitState")->stop();

   /* initiate events */
   stopwatch::instance("InitEvent")->start();
   LOG_IF(INFO, event_com<App>::instance()->rank() == 0) << "Initiate events";

   /* parse local event */
   ev_vec<App> parsed_events;
   app_.init_events(parsed_events,
                    event_com<App>::instance()->rank(),
                    event_com<App>::instance()->rank_size());

   /* send to target lp */
   ev_vec<App> initial_events;
   bool sh_wait = true;
   event_com<App>::instance()->shuffle(sh_wait,
                                       initial_events,
                                       parsed_events,
                                       lp_mngr<App>::instance()->partition());
   while (sh_wait);

   for (auto it = initial_events.begin(); it != initial_events.end(); ++it) {
     lp<App>* lp_;
     lp_mngr<App>::instance()->get_lp(lp_, (*it)->destination());
     lp_->init_event(*it);
   }

   long num_events = initial_events.size();
   long byte_events = sizeof(event<App>);
   long sum_events;
   bool rd_wait = true;
   event_com<App>::instance()->reduce_sum(rd_wait, sum_events, num_events);
   while (rd_wait);
   stopwatch::instance("InitEvent")->stop();

   /* input data information */
   LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
     << "\n====================================================================\n"
     << " Number of partitions: " << event_com<App>::instance()->rank_size() << "\n"
     << " Thread pool size: " << App::thr_pool_size() << "\n\n"
     << " Total initial events size " << (sum_events * byte_events) << " byte\n"
     << "     Total events num: " << sum_events << "\n"
     << "     One event size: " << byte_events << " byte\n"
     << " Total states size: " << (sum_states * byte_state) << " byte\n"
     << "     Total states num: " << sum_states << "\n"
     << "     One state size: " << byte_state << " byte\n\n"
     << " Total init time: "
     << stopwatch::instance("InitApp")->time_ms()
         + stopwatch::instance("InitState")->time_ms()
         + stopwatch::instance("InitEvent")->time_ms() << " ms\n"
     << "    Init application time: " << stopwatch::instance("InitApp")->time_ms() << " ms\n"
     << "    Init events time: " << stopwatch::instance("InitEvent")->time_ms() << " ms\n"
     << "    Init states time: " << stopwatch::instance("InitState")->time_ms() << " ms\n"
     << "====================================================================\n";
   event_com<App>::instance()->start();
}; /* init() */

template <class App>
void runner<App>::finish() {
  gvt_com<App>::del_instance();
  event_com<App>::instance()->stop();

  bool wait = true;
  event_com<App>::instance()->barrier(wait);
  while (wait);

  /* Sum up event processing elapsed time in all ranks */
  long ev_prcss_time_sum = 0;
  event_com<App>::instance()->reduce_sum(wait,
                                         ev_prcss_time_sum,
                                         stopwatch::instance("EventProcessing")
                                             ->time_ms());
  while (wait);
  /* Sum up # of processing events */
  long num_ev_prcss_sum = 0;
  event_com<App>::instance()->reduce_sum(wait,
                                         num_ev_prcss_sum,
                                         counter::sum("Events"));
  while (wait);
  /* Sum up # of cancel events */
  long num_cancel_sum = 0;
  event_com<App>::instance()->reduce_sum(wait,
                                         num_cancel_sum,
                                         counter::sum("Cancels"));
  while (wait);
  /* Sum up # of outputted events */
  long num_output_events_sum = 0;
  event_com<App>::instance()->reduce_sum(wait,
                                         num_output_events_sum,
                                         counter::sum("OutputtedEvent"));
  while (wait);
  /* Sum up Memory usage of Event */
  long mem_usage_ev_sum = 0;
  int size_ev_ = sizeof(event<App>);
  event_com<App>::instance()->reduce_sum(wait,
                                         mem_usage_ev_sum,
                                         size_ev_ * counter::sum("Events"));
  while (wait);
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0) << "Finish Simulation";
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
    << "\n====================================================================\n"
    << " Simulation time: " << stopwatch::instance("Simulation")->time_ms() << " ms\n"
    << "     Total Event Processing Elapsed time       : " << ev_prcss_time_sum / event_com<App>::instance()->rank_size() << " ms\n"
    << "     Average Event Processing Elapsed time     : " << ((float) ev_prcss_time_sum) / ((float) num_output_events_sum) / ((float) event_com<App>::instance()->rank_size())<< " ms/event \n"
    << "     Partition to Partition Communication time : " << " ms/partition \n"
    << "     Global Synchronization time               : " << " ms/partition \n\n"
    << " # of Global synchronizations   : " << counter::sum("GVT") << "\n"
    << " # of Processing Events         : " << num_ev_prcss_sum << "\n"
    << " # of Cancels (= # of Rollback) : " << num_cancel_sum << "\n"
    << " # of Outputted Events          : " << num_output_events_sum << "\n\n"
    << " Approximate total memory usage           : " << "\n"
    << "     Approximate memory usage for events  : " << mem_usage_ev_sum << " byte \n"
    << "     Approximate memory usage for cancels : " << "\n"
    << "     Approximate memory usage for states  : " << "\n\n"
    << " Approximate total store usage       : " << "\n"
    << "     Approximate store events usage  : " << "\n"
    << "     Approximate store cancels usage : " << "\n"
    << "     Approximate store states usage  : " << "\n"
    << "====================================================================\n";
  wait = true;
  event_com<App>::instance()->barrier(wait);
  while (wait);

  int rank_  = event_com<App>::instance()->rank();
  event_com<App>::instance()->finish();
  event_com<App>::del_instance();
  lp_mngr<App>::instance()->delete_lps(rank_);
  lp_mngr<App>::del_instance();
}; /* finish() */

template<class App>
void runner<App>::init_repeat() {
  stopwatch::instance("InitApp")->start();
  /* initiate application */
  app_.init();

  /* initiate communication manager */
  event_com<App>::instance()->init(lp_mngr<App>::instance());

  LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
      << "Initiate application";
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
      << "Initiate lp manager";
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
      << "Initiate event communicator ";

  /* initiate logical processes */
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0) << "Initiate lps";
  lp_mngr<App>::instance()->init_partition(app_.init_partition_index());
  lp_mngr<App>::instance()->init_lps(event_com<App>::instance()->rank(),
                                     event_com<App>::instance()->rank_size());
  stopwatch::instance("InitApp")->stop();

  /* initiate what-if event & state */
  stopwatch::instance("InitWhatIf")->start();
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
      << "Initiate what-if events & states";
  std::vector<boost::shared_ptr<const what_if<App> > > parsed_what_ifs_;
  app_.init_what_if(parsed_what_ifs_,
                    event_com<App>::instance()->rank(),
                    event_com<App>::instance()->rank_size());

  /* shuffle what_if query */
  bool wait_ = true;
  std::vector<boost::shared_ptr<const what_if<App> > > what_ifs_;
  event_com<App>::instance()->shuffle(wait_,
                                      what_ifs_,
                                      parsed_what_ifs_,
                                      lp_mngr<App>::instance()->partition());
  while(wait_);

  long num_events = 0; long num_states = 0;
  for (auto it_wi_ = what_ifs_.begin(); it_wi_ != what_ifs_.end(); ++it_wi_) {
    if ((*it_wi_)->update_state_.id() != -1) {
      /* initiate state */
      lp<App>* lp_;
      lp_mngr<App>::instance()->get_lp(lp_, (*it_wi_)->lp_id_);
      timestamp tmstmp_((*it_wi_)->time_, 0);
      st_ptr<App> init_state_
          = boost::make_shared<state<App> >((*it_wi_)->update_state_);
      lp_->init_state(init_state_, tmstmp_);
      ++num_states;

      /* load and initiate events */
      ev_vec<App> load_events_;
      lp_->load_events(load_events_, tmstmp_);
      for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
        lp_->init_event(*it);
        ++num_events;
      }

      /* load and initiate cancels */
      ev_vec<App> load_cancels_;
      lp_->load_cancels(load_cancels_, tmstmp_);
      for (auto it = load_cancels_.begin(); it != load_cancels_.end(); ++it) {
        lp_->set_cancel(*it);
      }
    }

    if ((*it_wi_)->delete_event_id_ != -1) {
      /* decide lp and time of what-if the query */
      lp<App>* lp_;
      lp_mngr<App>::instance()->get_lp(lp_, (*it_wi_)->lp_id_);
      timestamp tmstmp_((*it_wi_)->time_, (*it_wi_)->delete_event_id_);

      /* load and initiate events and cancels */
      ev_vec<App> load_events_;
      lp_->load_events(load_events_, tmstmp_);
      for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
        lp_->init_event(*it);
        ++num_events;
      }
      ev_vec<App> load_cancels_;
      lp_->load_cancels(load_cancels_, tmstmp_);
      for (auto it = load_cancels_.begin(); it != load_cancels_.end(); ++it) {
        lp_->set_cancel(*it);
      }

      /* delete the event and cancel */
      lp_->delete_ev(tmstmp_);
      lp_->delete_can(tmstmp_);

      /* load and initiate previous state */
      st_ptr<App> load_state_;
      timestamp load_tmstmp_;
      lp_->load_prev_state(load_state_, load_tmstmp_, tmstmp_);
      lp_->init_state(load_state_, load_tmstmp_);
      ++num_states;
    }

    if ((*it_wi_)->add_event_.id() != -1) {
      /* decide lp and time */
      lp<App>* lp_;
      lp_mngr<App>::instance()->get_lp(lp_, (*it_wi_)->lp_id_);
      timestamp tmstmp_((*it_wi_)->time_, (*it_wi_)->add_event_.id());

      /* load and initiate events */
      ev_vec<App> load_events_;
      lp_->load_events(load_events_, tmstmp_);
      for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
        lp_->init_event(*it);
        ++num_events;
      }
      ev_vec<App> load_cancels_;
      lp_->load_cancels(load_cancels_, tmstmp_);
      for (auto it = load_cancels_.begin(); it != load_cancels_.end(); ++it) {
        lp_->set_cancel(*it);
      }

      /* add events */
      ev_ptr<App> init_event_
          = boost::make_shared<event<App> >((*it_wi_)->add_event_);
      lp_->init_event(init_event_);
      ++num_events;

      /* load previous state */
      st_ptr<App> load_state_;
      timestamp load_tmstmp_;
      lp_->load_prev_state(load_state_, load_tmstmp_, tmstmp_);
      lp_->init_state(load_state_, load_tmstmp_);
      ++num_states;
    }
  }

  int byte_state = sizeof(state<App>);
  bool wait = true;
  long sum_states = 0;
  event_com<App>::instance()->reduce_sum(wait, sum_states, num_states);
  while (wait);

  int byte_events = sizeof(event<App>);
  long sum_events;
  wait = true;
  event_com<App>::instance()->reduce_sum(wait, sum_events, num_events);
  while (wait);

  stopwatch::instance("InitWhatIf")->stop();

  /* input data information */
  LOG_IF(INFO, event_com<App>::instance()->rank() == 0)
    << "\n====================================================================\n"
    << " Number of partitions: " << event_com<App>::instance()->rank_size() << "\n"
    << " Thread pool size: " << App::thr_pool_size() << "\n\n"
    << " Total initial events size " << (sum_events * byte_events) << " byte\n"
    << "     Total events num: " << sum_events << "\n"
    << "     One event size: " << byte_events << " byte\n"
    << " Total states size: " << (sum_states * byte_state) << " byte\n"
    << "     Total states num: " << sum_states << "\n"
    << "     One state size: " << byte_state << " byte\n\n"
    << " Total init time: "
    << stopwatch::instance("InitApp")->time_ms()
        + stopwatch::instance("InitState")->time_ms()
        + stopwatch::instance("InitEvent")->time_ms() << " ms\n"
    << "    Init application time: " << stopwatch::instance("InitApp")->time_ms() << " ms\n"
    << "    Init events time: " << stopwatch::instance("InitEvent")->time_ms() << " ms\n"
    << "    Init states time: " << stopwatch::instance("InitState")->time_ms() << " ms\n"
    << "====================================================================\n";

  event_com<App>::instance()->start();
}; /* init_repeat() */

template<class App>
void runner<App>::output(const timestamp& from, const timestamp& to) {
  timestamp fin_time(App::finish_time(), 0);
  timestamp to_ = std::min(to, fin_time);
  for (auto it = lp_mngr<App>::instance()->get_lps().begin();
       it != lp_mngr<App>::instance()->get_lps().end(); ++it) {
    it->second->std_out(from, to_);
  }
};

template<class App>
void runner<App>::run_lp_aggr(lp<App>* lp_, thr_pool* thr_pool) {
  /* send cancel event */
  ev_vec<App> cancels_;
  lp_->flush_buf(cancels_);
  for (auto it = cancels_.begin(); it != cancels_.end(); ++it) {
    event_com<App>::instance()->send_event(*it);
  }

  /* run application code */
  for (int i = 0; i < App::comm_interval(); ++i) {
    if (lp_->local_time() == timestamp::max()) {
      break;
    }
    ev_ptr<App> event;
    lp_->dequeue_event(event);
    if (!event) { break; }

    st_ptr<App> state;
    lp_->get_state(state);
    boost::optional<std::pair<ev_ptr<App>, st_ptr<App> > >
      update = app_.event_handler(event, state);
    if (!update || update->first->receive_time() > App::finish_time()) {
      break;
    } else { /* update ==  true */
      lp_->set_cancel(update->first);
      timestamp tmstp(update->first->send_time(), update->first->id());
      lp_->update_state(update->second, tmstp);
      event_com<App>::instance()->send_event(update->first);
    }
  }

  thr_pool->callback_end();
}; /* run_lp_aggr() */

template<class App>
void runner<App>::clear_old_ev_st(lp<App>* lp, thr_pool* pool,
                                  const timestamp& time) {
  lp->clear_old_ev(time);
  lp->clear_old_st(time);
  pool->callback_end();
};

template<class App>
void runner<App>::init_main(int* argc, char** argv[]) {
  /* how to use */
  std::string usage_message = "\n"
      "=====How to run simulation code====\n"
      "[Single Execution]\n"
      "    $ mpirun {BinaryCode} {InputFiles} \n"
      "[Differential Execution with file system]\n"
      "    $ mpirun {BinaryCode} --diff_init --sim_id=\"{id}\" --store_dir=\"{directory}\" {InputFiles}: (Initial Execution)\n"
      "    $ mpirun {BinaryCode} --diff_repeat --sim_id=\"{id}\" --store_dir=\"{directory}\" {InputFiles}: (Repeating Execution)\n"
      "[Differential Execution with remote store]\n"
      "    $ mpirun {BinaryCode} --diff_init --sim_id=\"{id}\" --store_ip=\"{ip address}\" {InputFiles}: (Initial Execution) \n"
      "    $ mpirun {BinaryCode} --diff_repeat --sim_id=\"{id}\" --store_ip=\"{ip address}\" {InputFiles}: (Repeating Execution)\n"
      "\n"
      "Exampale:\n"
      "    $ mpirun -np 4 ./build/TrafficSim ./traffic/ring/part/graph.part.4 ./traffic/ring/rd.sim.csv ./traffic/ring/trip.csv \n"
      "    $ mpirun -np 4 ./build/TrafficSim --diff_init --sim_id=99 --store_dir=\"./tmp\" ./traffic/ring/part/graph.part.4 ./traffic/ring/rd.sim.csv ./traffic/ring/trip.csv \n"
      "    $ mpirun -np 4 ./build/TrafficSim --diff_repeat --sim_id=99 --store_dir=\"./tmp\" ./traffic/ring/part/graph.part.4 ./traffic/ring/rd.sim.csv ./traffic/ring/trip.csv \n"
      "===================================";
  gflags::SetUsageMessage(usage_message);

  /* parse flags */
  gflags::ParseCommandLineFlags(argc, argv, true);

  /* init logs */
  FLAGS_logtostderr = 1;
  FLAGS_minloglevel = 0;
  google::InitGoogleLogging((*argv)[0]);
  google::InstallFailureSignalHandler();
};

template<class App>
void runner<App>::run() {
  runner<App>().simulation();
};

} /* namespace scalesim */
#endif /* SCALESIM_SIMULATION_SIM_RUNNER_HPP_ */
