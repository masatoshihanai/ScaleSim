/*
 * message_counter.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_UTIL_MESSAGE_COUNTER_HPP_
#define SCALESIM_UTIL_MESSAGE_COUNTER_HPP_

#include <map>
#include <boost/thread.hpp>

namespace scalesim {
class counter {
 private:
  counter(): count_(0){};
  explicit counter(long count): count_(count){};
  counter(const counter&);
  void operator=(const counter&);
 public:
  virtual ~counter(){};
 private:
  long count_;

 public:
  long value() const { return count_; };

  counter& operator++() {
    ++count_;
    return *this;
  };

  counter& operator--() {
    --count_;
    return *this;
  }

  counter& operator+=(long count) {
    count_ += count;
    return *this;
  };

  counter& operator-=(long count) {
    count_ -= count;
    return *this;
  };

 private:
  static std::map<std::string, std::vector<counter*> >* counters() {
    static std::map<std::string, std::vector<counter*> > counters_;
    return &counters_;
  };

 public:
  static counter* instance(std::string name) {
    if (counters()->find(name) == counters()->end()) {
      counters()->insert(
          std::pair<std::string, std::vector<counter*> >(
              name, std::vector<counter*>()));
    }
    counter* ret = new counter();
    counters()->find(name)->second.push_back(ret);
    return ret;
  };
  static long sum(std::string name) {
    long ret = 0;
    if (counters()->find(name) == counters()->end()) {
      return ret;
    }
    for (auto it = counters()->find(name)->second.begin();
        it != counters()->find(name)->second.end(); ++it) {
      ret += (*it)->value();
    }
    return ret;
  };
};

} /* namespace scalesim */

#endif /* SCALESIM_UTIL_MESSAGE_COUNTER_HPP_ */
