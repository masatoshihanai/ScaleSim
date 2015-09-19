/*
 * traffic_reader.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */

#ifndef TRAFFICSIM_TRAFFIC_READER_HPP_
#define TRAFFICSIM_TRAFFIC_READER_HPP_

#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "scalesim/util.hpp"
#include "traffic_sim.hpp"

class traffic_reader {
 private:
  traffic_reader(const traffic_reader&);
  void operator=(const traffic_reader&);
 private:
  traffic_reader() {};
  virtual ~traffic_reader() {};
 public:
  static void graph_read(parti_ptr ret_partition,
                         parti_indx_ptr ret_partition_index,
                         const std::string& file_path);

  static void road_read(st_vec<traffic_sim>& ret,
                        int rank,
                        int rank_size,
                        parti_ptr partition,
                        const std::string& file_path);

  static void trip_read(ev_vec<traffic_sim>& ret,
                        int rank,
                        int rank_size,
                        const std::string& file_path);

  static void what_if_read(std::vector<boost::shared_ptr<
                               const scalesim::what_if<traffic_sim> > >& ret,
                           const int rank,
                           const int rank_size,
                           const std::string& file_path);

}; /* class traffic_reader */

void traffic_reader::graph_read(parti_ptr ret_partition,
                                parti_indx_ptr ret_partition_index,
                                const std::string& file_path) {
  std::ifstream ifstream(file_path.c_str());
  if (ifstream.fail()) {
    ifstream.close();
    return;
  }
  std::string line;
  long id = 0;
  while (getline(ifstream, line)) {
    long partitionNum = atol(line.c_str());
    ret_partition_index->insert(std::pair<long, long>(partitionNum, id));
    ret_partition->push_back(partitionNum);
    id++;
  }
  ifstream.close();
};

void traffic_reader::road_read(st_vec<traffic_sim>& ret,
                               int rank,
                               int rank_size,
                               parti_ptr partition,
                               const std::string& file_path) {
  std::ifstream ifstream(file_path.c_str());
  if (ifstream.fail()) {
    ifstream.close();
    return;
  }
  std::string line;
  long id = 0;
  while (getline(ifstream, line)) {
    if ((*partition)[id]%rank_size == rank) {
      std::vector<std::string> cp;
      boost::split(cp, line, boost::is_any_of(";"));
      long id;
      std::vector<long> destinations(MAX_NUM_ROAD_PER_CP);
      std::vector<long> speed_limit(MAX_NUM_ROAD_PER_CP);
      std::vector<int> num_lanes(MAX_NUM_ROAD_PER_CP);
      std::vector<long> road_length(MAX_NUM_ROAD_PER_CP);
      int road_index = 0;
      for (auto road_it = cp.begin();
          road_it != cp.end(); ++road_it) {
        if (road_index > MAX_NUM_ROAD_PER_CP) {
          std::cerr << "# of roads per junction is BIGGER "
                       "than MAX_NUM_ROAD_PER_CP in " << id << std::endl;
        }
        std::vector<std::string> road;
        boost::split(road, *road_it, boost::is_any_of(","));
        id = atol(road[2].c_str());
        destinations[road_index] = atol(road[3].c_str());
        speed_limit[road_index] = atol(road[4].c_str());
        num_lanes[road_index] = atol(road[6].c_str());
        if (atol(road[5].c_str()) != 0) {
          road_length[road_index] = atol(road[5].c_str());
        } else {
          road_length[road_index] = (1 + atol(road[5].c_str()));
        }
        ++road_index;
      }
      ret.push_back(
          boost::make_shared<state<traffic_sim> >(
              state<traffic_sim>(id,
                                 destinations,
                                 speed_limit,
                                 num_lanes,
                                 road_length)));
    }
    id++;
  } /* while (getline(ifstream, line)) */
  ifstream.close();
  return;
}; /* load_read() */

void traffic_reader::trip_read(ev_vec<traffic_sim>& ret,
                               int rank,
                               int rank_size,
                               const std::string& file_path) {
  std::ifstream ifstream(file_path.c_str());
  if (ifstream.fail()) {
    ifstream.close();
    return;
  }
  long id = 0;
  std::string line;
  while (getline(ifstream, line)) {
    if (id % rank_size == rank) {
      std::vector<std::string> ev_str_;
      boost::split(ev_str_, line, boost::is_any_of(","));
      long vehicle_id = atol(ev_str_[1].c_str());
      long arrival_time = atol(ev_str_[3].c_str());
      long departure_time = atol(ev_str_[3].c_str());
      int track_counter = 0;
      int track_length = ev_str_.size() - 4;
      long path_tracks[MAX_PATH_LENGTH];
      if (track_length > MAX_PATH_LENGTH) {
        std::cerr << "!! the vehicle: " << vehicle_id
                  << " has track_length " << track_length
                  << " tracks, which is bigger than MAX_PATH_LENGTH "
                  << MAX_PATH_LENGTH << std::endl;
        track_length = MAX_PATH_LENGTH;
      }
      for (int i = 0; i < track_length; ++i) {
        path_tracks[i] = atol(ev_str_[i + 4].c_str());
      }
      ret.push_back(
          boost::make_shared<event<traffic_sim> >(
              event<traffic_sim>(vehicle_id,
                                 arrival_time,
                                 departure_time,
                                 path_tracks,
                                 track_counter,
                                 track_length)));
    }
    ++id;
  } /* while (getline(ifstream, line)) */
  ifstream.close();
  return;
}; /* trip_read() */

void traffic_reader::what_if_read(std::vector<boost::shared_ptr<
                                      const scalesim::what_if<traffic_sim> > >&
                                          ret,
                                  const int rank,
                                  const int rank_size,
                                  const std::string& file_path) {
  /* Read file */
  std::ifstream ifstream_(file_path);
  if (ifstream_.fail()) {
    ifstream_.close();
    LOG(INFO) << " Opening file " << file_path << " fails. Check file.";
    exit(1);
  }

  /* Read lines */
  long i = 0;
  std::string line;
  while (std::getline(ifstream_, line)) {
    if (i % rank_size != rank) {
      ++i;
      continue;
    }
    if (line.compare(0, 2, "SC") == 0) {
      /*
       * Query type is state change (SC).
       * Input format is like that.
       *   SC,{lp id},{time};{lp format};...;..;..
       * ex)
       *   SC,10,555;R,0,10,1,10,100,2;R,1,10,1,10,200,1;R,3,10,3,10,200,100
       */
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(";"));

      /* read lp id & time */
      std::vector<std::string> id_time_;
      boost::split(id_time_, vec_[0], boost::is_any_of(","));
      long lp_id = atol(id_time_[1].c_str());
      long time_ = atol(id_time_[2].c_str());

      /* read roads */
      std::vector<long> destinations(MAX_NUM_ROAD_PER_CP);
      std::vector<long> speed_limit(MAX_NUM_ROAD_PER_CP);
      std::vector<int> num_lanes(MAX_NUM_ROAD_PER_CP);
      std::vector<long> rd_length(MAX_NUM_ROAD_PER_CP);
      int road_index = 0;
      auto road_it = vec_.begin();
      ++road_it;
      for (;road_it != vec_.end(); ++road_it) {
        if (road_index > MAX_NUM_ROAD_PER_CP) {
          std::cerr << "# of roads per junction is BIGGER "
                    << "than MAX_NUM_ROAD_PER_CP in " << lp_id << std::endl;
        }
        std::vector<std::string> road;
        boost::split(road, *road_it, boost::is_any_of(","));
        destinations[road_index] = atol(road[3].c_str());
        speed_limit[road_index] = atol(road[4].c_str());
        num_lanes[road_index] = atol(road[6].c_str());
        if (atol(road[5].c_str()) != 0) {
          rd_length[road_index] = atol(road[5].c_str());
        } else {
          rd_length[road_index] = (1 + atol(road[5].c_str()));
        }
        ++road_index;
      }

      /* initiate what_if query */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id,time_,
                                             state<traffic_sim>(lp_id,
                                                                destinations,
                                                                speed_limit,
                                                                num_lanes,
                                                                rd_length)));

      /* push back this what if query */
      ret.push_back(wh_if_);
    } else if (line.compare(0, 2, "AE") == 0) {
      /*
       * Query type is add event (AE).
       * Input format is like that.
       *   AE,{event id},0,{Deptime},{1st cp},{2nd cp},...
       * ex)
       *   AE,0,0,1,0,1,2,3,4,5
       */
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(","));

      /* read lp id and time */
      long lp_id_ = atol(vec_[4].c_str());
      long time_ = atol(vec_[3].c_str());

      /* read adding event */
      long vehicle_id_ = atol(vec_[1].c_str());
      long arrival_time_ = atol(vec_[3].c_str());
      long departure_time_ = atol(vec_[3].c_str());
      int track_counter_ = 0;
      int length_ = vec_.size() - 4;
      long path_tracks_[MAX_PATH_LENGTH];
      if (length_ > MAX_PATH_LENGTH) {
        std::cerr << "!! the vehicle: " << vehicle_id_
                  << " has track_length " << length_
                  << " tracks, which is bigger than MAX_PATH_LENGTH "
                  << MAX_PATH_LENGTH << std::endl;
        length_ = MAX_PATH_LENGTH;
      }
      for (int i = 0; i < length_; ++i) {
        path_tracks_[i] = atol(vec_[i + 4].c_str());
      }

      /* initiate adding event */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
        = boost::make_shared<scalesim::what_if<traffic_sim> >(
            scalesim::what_if<traffic_sim>(lp_id_, time_,
                                           event<traffic_sim>(vehicle_id_,
                                                              arrival_time_,
                                                              departure_time_,
                                                              path_tracks_,
                                                              track_counter_,
                                                              length_)));

      /* push back to return */
      ret.push_back(wh_if_);
    } else if (line.compare(0,2,"DE") == 0) {
      /*
       * Query type is delete event (DE).
       * Input format is like that.
       *   DE, {lp id}, {time}, {event id}
       *   RE, {event id}, {source id}, {send time}, {destination id}, {receive time}
       * ex)
       *   DE,999,10,555
       *   RE,2,2,94,3,135
       */
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(","));
      /* read lp id, time, event id */
      long lp_id_ = atol(vec_[1].c_str());
      long time_ = atol(vec_[2].c_str());
      long ev_id_ = atol(vec_[3].c_str());

      /* initiate what-if query */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id_, time_, ev_id_));

      ret.push_back(wh_if_);
    } else if (line.compare(0,2,"RE") == 0) {
      std::vector<std::string> vec_;
      boost::split(vec_, line, boost::is_any_of(","));
      /* read lp id, time, event id */
      long lp_id_ = atol(vec_[4].c_str());
      long time_ = atol(vec_[5].c_str());
      long ev_id_ = atol(vec_[1].c_str());

      /* initiate what-if query */
      boost::shared_ptr<scalesim::what_if<traffic_sim> > wh_if_
          = boost::make_shared<scalesim::what_if<traffic_sim> >(
              scalesim::what_if<traffic_sim>(lp_id_, time_, ev_id_));

      ret.push_back(wh_if_);
    }
    ++i;
  } /* while (std::getline(ifstream_, line)) */
}; /* what_if_read*/

#endif /* TRAFFICSIM_TRAFFIC_READER_HPP_ */
