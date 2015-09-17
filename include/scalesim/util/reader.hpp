/*
 * reader.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_UTIL_READER_HPP_
#define SCALESIM_UTIL_READER_HPP_

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

namespace scalesim {

class graph_reader {
  typedef boost::shared_ptr<std::vector<long> > partition_ptr;
  typedef boost::shared_ptr<boost::unordered_multimap<long, long> > index_ptr;
 private:
  graph_reader(const graph_reader&);
  void operator=(const graph_reader&);
 public:
  graph_reader() {};
  virtual ~graph_reader() {};
 private:
  partition_ptr partition;
  index_ptr partition_index;
 public:
  bool read(const std::string& file_path, partition_ptr partition,
      index_ptr partition_index) {
    this->partition = partition;
    this->partition_index = partition_index;
    std::ifstream ifstream(file_path.c_str());
    if (ifstream.fail()) {
      ifstream.close();
      return false;
    }
    std::string line;
    long id = 0;
    while (getline(ifstream, line)) {
      long partitionNum = atol(line.c_str());
      partition_index->insert(std::pair<long, long>(partitionNum, id));
      partition->push_back(partitionNum);
      id++;
    }
    ifstream.close();
    return true;
  };
};

class state_reader {
 private:
  state_reader(const state_reader&);
  void operator=(const state_reader&);
 public:
  state_reader() {};
  virtual ~state_reader() {};
 private:
  std::vector<std::pair<long, std::string> > state_;
 public:
  bool read(const std::string file_path, int rank, int rank_size,
      boost::shared_ptr<std::vector<long> > partition) {
    std::ifstream ifstream(file_path.c_str());
    if (ifstream.fail()) {
      ifstream.close();
      return false;
    }
    std::string line;
    long id = 0;
    while (getline(ifstream, line)) {
      if ((*partition)[id]%rank_size == rank) {
        state_.push_back(std::pair<long, std::string>(id, line));
      }
      id++;
    }
    ifstream.close();
    return true;
  };
  std::vector<std::pair<long, std::string> >& states() {
    return state_;
  }
};

class event_reader {
 private:
  event_reader(const event_reader&);
  void operator=(const event_reader&);
 public:
  event_reader(){};
  virtual ~event_reader(){};
 private:
  std::vector<std::string> events_;
 public:
  bool read(std::string file_path, int rank, int rank_size) {
    std::ifstream ifstream(file_path.c_str());
    if (ifstream.fail()) {
      ifstream.close();
      return false;
    }
    long id = 0;
    std::string line;
    while (getline(ifstream, line)) {
      if (id % rank_size == rank) {
        events_.push_back(line);
      }
      ++id;
    }
    ifstream.close();
    return true;
  };
  std::vector<std::string>& events() {
    return events_;
  }
};

} /* namespace scalesim */
#endif /* SCALESIM_UTIL_READER_HPP_ */
