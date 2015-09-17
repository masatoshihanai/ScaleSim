/*
 * store_base.hpp
 *
 *  Copyright (c) 2015 Masatoshi Hanai
 *
 *  This software is released under MIT License.
 *  See LICENSE.
 *
 */
#ifndef SCALESIM_LOGICAL_PROCESS_STORE_STORE_BASE_HPP_
#define SCALESIM_LOGICAL_PROCESS_STORE_STORE_BASE_HPP_

#include "scalesim/logical_process/store/leveldb_store.hpp"
#include "scalesim/util.hpp"

namespace scalesim {

template<class Object> using db_store = leveldb_store<Object>;

template<class App>
class store {
 private:
  store(){};
  store(const store&);
  void operator=(const store&);
 private:
  db_store<event<App> > event_db_;
  db_store<event<App> > cancel_db_;
  db_store<state<App> > state_db_;
  static store<App>* instance() {
    static store* store_;
    if (store_ == NULL) {store_ = new store<App>(); }
    return store_;
  };
 public:
  static db_store<event<App> >* ev_store() {
    return &(instance()->event_db_);
  };

  static db_store<event<App> >* can_store() {
    return &(instance()->cancel_db_);
  };

  static db_store<state<App> >* st_store() {
    return &(instance()->state_db_);
  };

  static void init(long rank) {
    store<App>::ev_store()->init("event", rank);
    store<App>::can_store()->init("cancel", rank);
    store<App>::st_store()->init("state", rank);
  };

  static void init_read(long rank) {
    store<App>::ev_store()->init_read("event", rank);
    store<App>::can_store()->init_read("cancel", rank);
    store<App>::st_store()->init_read("state", rank);
  };

  /*
   * delete database connection (keep data)
   */
  static void finish() {
    store::ev_store()->finish();
    store::can_store()->finish();
    store::st_store()->finish();
  };

  /*
   * delete database files for test
   */
  static void clear() {
    store::ev_store()->clear_db();
    store::can_store()->clear_db();
    store::st_store()->clear_db();
  };
};

} /* namespace scalesim */

#endif /* SCALESIM_LOGICAL_PROCESS_STORE_STORE_BASE_HPP_ */
