/*
 * stopwatch.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_UTIL_STOPWATCH_HPP_
#define SCALESIM_UTIL_STOPWATCH_HPP_

#include <glog/logging.h>
#include <sys/time.h>
#include <map>

namespace scalesim {

class stopwatch {
 private:
  stopwatch():time(0) {};
  stopwatch(const stopwatch&);
  void operator=(const stopwatch&);
 public:
  virtual ~stopwatch(){};
 private:
  int time;
  struct timeval start_time;
 public:
  void start() {
    gettimeofday(&start_time, NULL);
  };
  void stop() {
    struct timeval now;
    gettimeofday(&now, NULL);
    time += (now.tv_sec - start_time.tv_sec) * 1000
        + (now.tv_usec - start_time.tv_usec) / 1000;
  };
  void reset() { time = 0; };
  int time_ms() { return time; };

 private:
  static std::map<std::string, stopwatch*>* stopwatches() {
    static std::map<std::string, stopwatch*> stopwtches;
    return &stopwtches;
 }
 public:
  static stopwatch* instance(std::string name) {
    if (stopwatches()->find(name) == stopwatches()->end()) {
      stopwatches()->insert(std::pair<std::string, stopwatch*>(name, new stopwatch()));
    }
    return stopwatches()->find(name)->second;
  };
};

} /* namespace scalesim */

#endif /* SCALESIM_UTIL_STOPWATCH_HPP_ */
