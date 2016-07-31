/*
 * type.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_UTIL_TYPE_HPP_
#define SCALESIM_UTIL_TYPE_HPP_

#include <vector>
#include <boost/unordered_set.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

template<class App> using event = typename App::Event;
template<class App> using state = typename App::State;

template<class App> using ev_ptr = boost::shared_ptr<const event<App> >;
template<class App> using st_ptr = boost::shared_ptr<const state<App> >;

template<class App> using ev_vec = std::vector<ev_ptr<App> >;
template<class App> using st_vec = std::vector<st_ptr<App> >;

/* partition is a table from id to partition number */
using parti_ptr = boost::shared_ptr<std::vector<long> >;
/* partition index is a table from partition to involving ids */
using parti_indx = boost::unordered_multimap<long, long>;
using parti_indx_ptr = boost::shared_ptr<parti_indx>;

#endif /* SCALESIM_UTIL_TYPE_HPP_ */
