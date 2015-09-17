/*
 * timestamp.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_UTIL_TIMESTAMP_HPP_
#define SCALESIM_UTIL_TIMESTAMP_HPP_

#include <boost/serialization/access.hpp>

namespace scalesim {

class timestamp {
 public:
  timestamp(): time_(-1), id_(-1){};
  timestamp(const timestamp& origin) {
    time_ = origin.time_;
    id_ = origin.id_;
  };
  timestamp(long time, long id):
    time_(time), id_(id) {
  };
  void operator=(const timestamp& right) {
    time_ = right.time_;
    id_ = right.id_;
  };
 private:
  long time_;
  long id_;
 public:
  long time() const { return time_; };
  long id() const { return id_; };
  void update(long time, long id) {
    time_ = time;
    id_ = id;
  };
  void update(const timestamp& tmstmp_) {
    time_ = tmstmp_.time_;
    id_ = tmstmp_.id_;
  };
  bool operator==(const timestamp& right) const {
    if (time_ == right.time_ && id_ == right.id_) return true;
    return false;
  };
  bool operator!=(const timestamp& right) const {
    if (*this == right) return false;
    return true;
  };
  bool operator<(const timestamp& right) const {
    if (time_ < right.time_) { return true; }
    if (time_ > right.time_) { return false; }
    /* time_ == right.time_ */
    if (id_ < right.id_) { return true; }
    return false;
  };
  bool operator>(const timestamp& right) const {
    if (time_ > right.time_) { return true; }
    if (time_ < right.time_) { return false; }
    /* time_ == right.time_ */
    if (id_ > right.id_) { return true; }
    return false;
  };
  static timestamp max() {
    return timestamp(std::numeric_limits<long>::max(), std::numeric_limits<long>::max());
  };
  static timestamp zero() {
    return timestamp(0, 0);
  };
  static timestamp null() {
    return timestamp(-1, -1);
  };

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, unsigned int version) {
    ar & time_;
    ar & id_;
  }
};

} /* namespace scalesim */

#endif /* SCALESIM_UTIL_TIMESTAMP_HPP_ */
