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
  event_com<App> com_;
  gvt_com<App> gvt_;
  lp_mngr<App> lp_mngr_;
  thr_pool pool;
  std::vector<scheduler<App> > schedulers_;
  long byte_events, byte_state;
 private:
  runner(): pool(App::num_thr()),
            schedulers_(App::num_thr()),
            byte_events(0),
            byte_state(0){};
  virtual ~runner(){};
  runner(const runner&);
  void operator=(const runner&);

 private:
  int simulation();

  void init();
  void init_repeat();
  void loop();
  void finish();

  void run(int scheduler_id);
  void run_lp_aggr(lp<App>* lp);
  void clear(int scheduler_id, const timestamp& time);

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

  LOG_IF(INFO, com_.rank() == 0) << "Start Simulation";
  stopwatch::instance("Simulation")->start();
  loop();
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
   com_.init(&lp_mngr_);

   LOG_IF(INFO, com_.rank() == 0)
       << "Initiate application";
   LOG_IF(INFO, com_.rank() == 0)
       << "Initiate lp manager";
   LOG_IF(INFO, com_.rank() == 0)
       << "Initiate com  manager";

   /* initiate logical processes */
   LOG_IF(INFO, com_.rank() == 0) << "Initiate lps";
   lp_mngr_.init_partition(app_.init_partition_index());
   lp_mngr_.init_lps(com_.rank(),com_.rank_size());
   lp_mngr_.init_scheduler(&schedulers_);

   stopwatch::instance("InitApp")->stop();

   /* initiate state */
   stopwatch::instance("InitState")->start();
   LOG_IF(INFO, com_.rank() == 0) << "Initiate states";
   st_vec<App> states;
   app_.init_states_in_this_rank(states,
                                 com_.rank(),
                                 com_.rank_size(),
                                 lp_mngr_.partition());
   for (auto it = states.begin(); it != states.end(); ++it) {
     byte_state = (*it)->size() > byte_state ? (*it)->size(): byte_state;
     lp<App>* lp_;
     lp_mngr_.get_lp(lp_, (*it)->id());
     lp_->init_state(*it);
   }
   bool wait = true;
   long sum_states = 0;
   com_.reduce_sum(wait, sum_states, states.size());
   while (wait);

   stopwatch::instance("InitState")->stop();

   /* initiate events */
   stopwatch::instance("InitEvent")->start();
   LOG_IF(INFO, com_.rank() == 0) << "Initiate events";

   /* parse local event */
   ev_vec<App> parsed_events;
   app_.init_events(parsed_events,
                    com_.rank(),
                    com_.rank_size());

   /* send to target lp */
   ev_vec<App> initial_events;
   bool sh_wait = true;
   com_.shuffle(sh_wait, initial_events, parsed_events, lp_mngr_.partition());
   while (sh_wait);

   for (auto it = initial_events.begin(); it != initial_events.end(); ++it) {
     byte_events = byte_events < (*it)->size()? (*it)->size(): byte_events;
     lp<App>* lp_;
     lp_mngr_.get_lp(lp_, (*it)->destination());
     lp_->init_event(*it);
   }

   long num_events = initial_events.size();
   long sum_events;
   bool rd_wait = true;
   com_.reduce_sum(rd_wait, sum_events, num_events);
   while (rd_wait);
   stopwatch::instance("InitEvent")->stop();

   /* input data information */
   LOG_IF(INFO, com_.rank() == 0)
     << "\n====================================================================\n"
     << " Number of partitions: " << com_.rank_size() << "\n"
     << " Number of threads: " << App::num_thr() << "\n\n"
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
   com_.start();
}; /* init() */

template<class App>
void runner<App>::init_repeat() {
  stopwatch::instance("InitApp")->start();
  /* initiate application */
  app_.init();

  /* initiate communication manager */
  com_.init(&lp_mngr_);

  LOG_IF(INFO, com_.rank() == 0)
      << "Initiate application";
  LOG_IF(INFO, com_.rank() == 0)
      << "Initiate lp manager";
  LOG_IF(INFO, com_.rank() == 0)
      << "Initiate event communicator ";

  /* initiate logical processes */
  LOG_IF(INFO, com_.rank() == 0) << "Initiate lps";
  lp_mngr_.init_partition(app_.init_partition_index());
  lp_mngr_.init_lps(com_.rank(), com_.rank_size());
  lp_mngr_.init_scheduler(&schedulers_);

  stopwatch::instance("InitApp")->stop();

  /* initiate what-if event & state */
  stopwatch::instance("InitWhatIf")->start();
  LOG_IF(INFO, com_.rank() == 0)
      << "Initiate what-if events & states";
  std::vector<boost::shared_ptr<const what_if<App> > > parsed_what_ifs_;
  app_.init_what_if(parsed_what_ifs_, com_.rank(), com_.rank_size());

  /* shuffle what_if query */
  bool wait_ = true;
  std::vector<boost::shared_ptr<const what_if<App> > > what_ifs_;
  com_.shuffle(wait_, what_ifs_, parsed_what_ifs_, lp_mngr_.partition());
  while(wait_);

  long num_events = 0; long num_states = 0;
  for (auto it_wi_ = what_ifs_.begin(); it_wi_ != what_ifs_.end(); ++it_wi_) {
    if ((*it_wi_)->update_state_.id() != -1) {
      /* initiate state */
      byte_state = (*it_wi_)->update_state_.size() > byte_state ?
                     (*it_wi_)->update_state_.size(): byte_state;
      lp<App>* lp_;
      lp_mngr_.get_lp(lp_, (*it_wi_)->lp_id_);
      timestamp tmstmp_((*it_wi_)->time_, 0);
      st_ptr<App> init_state_
          = boost::make_shared<state<App> >((*it_wi_)->update_state_);
      lp_->init_state(init_state_, tmstmp_);
      ++num_states;

      /* load and initiate events */
      ev_vec<App> load_events_;
      lp_->load_events(load_events_, tmstmp_);
      for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
        byte_events = byte_events < (*it)->size()? (*it)->size(): byte_events;
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
      lp_mngr_.get_lp(lp_, (*it_wi_)->lp_id_);
      timestamp tmstmp_((*it_wi_)->time_, (*it_wi_)->delete_event_id_);

      /* load and initiate events and cancels */
      ev_vec<App> load_events_;
      lp_->load_events(load_events_, tmstmp_);
      for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
        byte_events = byte_events < (*it)->size()? (*it)->size(): byte_events;
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
      byte_state
          = load_state_->size() > byte_state ? load_state_->size(): byte_state;
      ++num_states;
    }

    if ((*it_wi_)->add_event_.id() != -1) {
      /* decide lp and time */
      lp<App>* lp_;
      lp_mngr_.get_lp(lp_, (*it_wi_)->lp_id_);
      timestamp tmstmp_((*it_wi_)->time_, (*it_wi_)->add_event_.id());

      /* load and initiate events */
      ev_vec<App> load_events_;
      lp_->load_events(load_events_, tmstmp_);
      for (auto it = load_events_.begin(); it != load_events_.end(); ++it) {
        byte_events = byte_events < (*it)->size()? (*it)->size(): byte_events;
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
      timestamp time_(init_event_->receive_time(), init_event_->id());
      ++num_events;

      /* load previous state */
      st_ptr<App> load_state_;
      timestamp load_tmstmp_;
      lp_->load_prev_state(load_state_, load_tmstmp_, tmstmp_);
      lp_->init_state(load_state_, load_tmstmp_);
      byte_state
          = load_state_->size() > byte_state ? load_state_->size(): byte_state;
      ++num_states;
    }
  }

  bool wait = true;
  long sum_states = 0;
  com_.reduce_sum(wait, sum_states, num_states);
  while (wait);

  long sum_events;
  wait = true;
  com_.reduce_sum(wait, sum_events, num_events);
  while (wait);

  stopwatch::instance("InitWhatIf")->stop();

  /* input data information */
  LOG_IF(INFO, com_.rank() == 0)
    << "\n====================================================================\n"
    << " Number of partitions: " << com_.rank_size() << "\n"
    << " Number of Threads: " << App::num_thr() << "\n\n"
    << " Total events num: " << sum_events << "\n"
    << " Total states num: " << sum_states << "\n"
    << " Total init time: "
    << stopwatch::instance("InitApp")->time_ms()
        + stopwatch::instance("InitState")->time_ms()
        + stopwatch::instance("InitEvent")->time_ms() << " ms\n"
    << "    Init application time: " << stopwatch::instance("InitApp")->time_ms() << " ms\n"
    << "    Init events time: " << stopwatch::instance("InitEvent")->time_ms() << " ms\n"
    << "    Init states time: " << stopwatch::instance("InitState")->time_ms() << " ms\n"
    << "====================================================================\n";

  com_.start();
}; /* init_repeat() */

template<class App>
void runner<App>::loop() {
  while (true) {
    /* processing events with threads */
    stopwatch::instance("EventProcessing")->start();
    for (int i = 0; i < schedulers_.size(); ++i) {
      pool.post(boost::bind(&runner<App>::run, this, i));
    }
    pool.wait_all();
    stopwatch::instance("EventProcessing")->stop();

    /* compute minimum local time */
    timestamp local_min_time_ = timestamp::max();
    for (auto it = schedulers_.begin(); it != schedulers_.end(); ++it) {
      local_min_time_ = std::min(local_min_time_, it->min_locals());
    }
    gvt_.update_local_min(local_min_time_);
    gvt_.increment_gsync_interval();

    if (gvt_.check_updated()) {
      /* output results */
      for (int i = 0; i < schedulers_.size(); ++i) {
        for (auto it = schedulers_[i].active_lp()->begin();
             it != schedulers_[i].active_lp()->end(); ++it) {
          lp<App>* lp_;
          lp_mngr_.get_lp(lp_, *it);
          lp_->std_out(std::min(gvt_.gtime(), timestamp(App::finish_time(),0)));
        }
      }

      /* clear(store) events/states */
      stopwatch::instance("Clear")->start();
      for (int i = 0; i < schedulers_.size(); ++i) {
        pool.post(boost::bind(&runner<App>::clear, this, i, gvt_.gtime()));
      }
      pool.wait_all();
      stopwatch::instance("Clear")->stop();
    }

    /* finish */
    if (gvt_.gtime().time() >= App::finish_time()) {
      break;
    }
  } /* while loop */
}; /* loop() */

template <class App>
void runner<App>::finish() {
  com_.stop();

  bool wait = true;
  com_.barrier(wait);
  while (wait);

  /* Sum up event processing elapsed time in all ranks */
  long ev_prcss_time_sum = 0;
  com_.reduce_sum(wait, ev_prcss_time_sum,
                  stopwatch::instance("EventProcessing")->time_ms());
  while (wait);
  /* Sum up communication time */
  long communication_time_ = 0;
  com_.reduce_sum(wait, communication_time_,
                  stopwatch::instance("PartiToPartiCommunication")->time_ms());
  while (wait);
  /* Sum up gloabl synchronization time */
  long global_sync_time_ = 0;
  com_.reduce_sum(wait, global_sync_time_,
                  stopwatch::instance("GlobalSync")->time_ms());
  while(wait);
  /* Sum up clear events, cancels & states time */
  long clear_objects_time_ = 0;
  com_.reduce_sum(wait, clear_objects_time_,
                  stopwatch::instance("Clear")->time_ms());
  while(wait);
  /* Sum up # of processing events */
  long num_ev_prcss_sum = counter::sum("Events");
  com_.reduce_sum(wait, num_ev_prcss_sum, num_ev_prcss_sum);
  while (wait);
  /* Sum up # of cancel events */
  long num_cancel_sum = counter::sum("Cancels");
  com_.reduce_sum(wait, num_cancel_sum, num_cancel_sum);
  while (wait);
  /* Sum up # of outputted events */
  long num_output_events_sum = counter::sum("OutputtedEvent");
  long store_usage_ev_ = num_output_events_sum * byte_events;
  com_.reduce_sum(wait, num_output_events_sum, num_output_events_sum);
  while (wait);
  /* Sum up store usage of events */
  com_.reduce_sum(wait, store_usage_ev_, store_usage_ev_);
  while (wait);

  /* Sum up store usage of State */
  long store_usage_st_ = byte_state * counter::sum("StateClear");
  com_.reduce_sum(wait, store_usage_st_, store_usage_st_);
  while (wait);

  LOG_IF(INFO, com_.rank() == 0) << "Finish Simulation";
  LOG_IF(INFO, com_.rank() == 0)
    << "\n====================================================================\n"
    << " Simulation elapsed time: " << stopwatch::instance("Simulation")->time_ms() << " ms\n"
    << "     Average all events processing time   : " << ev_prcss_time_sum / com_.rank_size() << " ms/partition\n"
    << "     Average one event processing time    : " << ((float) ev_prcss_time_sum) / ((float) num_output_events_sum) / ((float) com_.rank_size())<< " ms/event \n"
    << "     Average comm time between partitions : " << communication_time_ / com_.rank_size() << " ms/partition \n"
    << "     Global synchronization time          : " << global_sync_time_ / com_.rank_size() << " ms/partition \n"
    << "     Clear(store) object time             : " << clear_objects_time_ / com_.rank_size() << " ms/partition \n"
    << "\n"
    << " # of global synchronizations   : " << counter::sum("GVT") << "\n"
    << " # of processing events         : " << num_ev_prcss_sum << "\n"
    << " # of cancels (= # of rollback) : " << num_cancel_sum << "\n"
    << " # of output events             : " << num_output_events_sum << "\n\n"
    << " Approximate total store usage       : " << store_usage_ev_*2 + store_usage_st_ << " byte \n"
    << "     Approximate store events usage  : " << store_usage_ev_ << " byte \n"
    << "     Approximate store cancels usage : " << store_usage_ev_ << " byte \n"
    << "     Approximate store states usage  : " << store_usage_st_ << " byte \n"
    << "====================================================================\n";
  wait = true;
  com_.barrier(wait);
  while (wait);

  int rank_  = com_.rank();
  com_.finish();
  lp_mngr_.delete_lps(rank_);
}; /* finish() */

template<class App>
void runner<App>::run(int scheduler_id) {
  for (int i = 0; i < App::gsync_interval(); ++i) {
    long lp_id_ = schedulers_[scheduler_id].dequeue();
    if (lp_id_ < 0) break;
    lp<App>* lp_;
    lp_mngr_.get_lp(lp_, lp_id_);
    run_lp_aggr(lp_);
    schedulers_[scheduler_id].queue(lp_->local_time(), lp_->id());
  }
  pool.callback_end();
};

template<class App>
void runner<App>::run_lp_aggr(lp<App>* lp_) {
  /* send cancel event */
  ev_vec<App> cancels_;
  lp_->flush_buf(cancels_);
  for (auto it = cancels_.begin(); it != cancels_.end(); ++it) {
    com_.send_event(*it);
  }

  /* run application code */
  for (int i = 0; i < App::switch_lp_interval(); ++i) {
    if (lp_->local_time() == timestamp::max()) {
      break;
    }
    ev_ptr<App> event;
    lp_->dequeue_event(event);
    if (!event) { break; }

    st_ptr<App> state;
    lp_->get_state(state);
    boost::optional<std::pair<std::vector<ev_ptr<App> >, st_ptr<App> > >
        update = app_.event_handler(event, state);
    if (!update) {
      break;
    } else { /* update ==  true */
      for (auto it = update->first.begin(); it != update->first.end(); ++it) {
        lp_->set_cancel(*it);
        timestamp tmstp((*it)->send_time(), (*it)->id());
        lp_->update_state(update->second, tmstp);
        com_.send_event(*it);
      }
    }
  }
}; /* run_lp_aggr() */

template<class App>
void runner<App>::clear(int scheduler_id, const timestamp& time) {
  for (auto it = schedulers_[scheduler_id].active_lp()->begin();
      it != schedulers_[scheduler_id].active_lp()->end(); ++it) {
    lp<App>* lp_;
    lp_mngr_.get_lp(lp_, *it);
    lp_->clear_old_ev(time);
    lp_->clear_old_st(time);
  }
  //schedulers_[scheduler_id].reset_active_lp();
  pool.callback_end();
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
