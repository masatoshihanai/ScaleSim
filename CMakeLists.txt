string (ASCII 27 Esc)
set (ColorReset "${Esc}[m")
set (Color "${Esc}[1;35m")

cmake_minimum_required (VERSION 2.6)
project (ScaleSim)

### Compiler ###
set (CMAKE_CXX_COMPILER mpic++)
add_compile_options(-std=c++11)

### Compiling mode ###
set (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O0")
set (CMAKE_CXX_FLAGS_DEBUG "-g -O0")

### Library ###
find_library (BOOST_MPI NAMES boost_mpi boost_mpi-mt
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (BOOST_MPI)
    message (STATUS "boost_mpi ${BOOST_MPI} ${Color}ok${ColorReset}")
else ()
    message (STATU "boost_mpi NG")
endif ()

find_library (BOOST_THREAD NAMES boost_thread-mt boost_thread
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (BOOST_THREAD)
    message (STATUS "boost_thread ${BOOST_THREAD} ${Color}ok${ColorReset}")
else ()
    message (STATUS "boost_thread NG")
endif ()

find_library (BOOST_SYSTEM NAMES boost_system-mt boost_system
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (BOOST_SYSTEM)
    message (STATUS "boost_system ${BOOST_SYSTEM} ${Color}ok${ColorReset}")
else ()
    message (STATUS "boost_system NG")
endif ()

find_library (BOOST_SERIALIZATION NAMES boost_serialization-mt boost_serialization
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (BOOST_SERIALIZATION)
    message (STATUS "boost_serialization ${BOOST_SERIALIZATION} ${Color}ok${ColorReset}")
else ()
    message (STATUS "boost_serialization NG")
endif ()

find_library (BOOST_FILESYSTEM NAMES boost_filesystem-mt boost_filesystem
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (BOOST_FILESYSTEM)
    message (STATUS "boost_filesystem ${BOOST_FILESYSTEM} ${Color}ok${ColorReset}")
else ()
    message (STATUS "boost_filesystem NG")
endif ()

find_library (LEVEL_DB NAMES leveldb
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (LEVEL_DB)
    message (STATUS "level_db ${LEVEL_DB} ${Color}ok${ColorReset}")
else ()
    message (STATUS "level_db NG")
endif ()

find_library (BERKELEY_DB NAMES db_cxx
    PATH ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (Berkeley_DB)
    message (STATUS "berkeley_db ${BERKELEY_DB} ${Color}ok${ColorReset}")
else ()
    message (STATUS "berkeley_db NG")
endif()

find_library (REDIS NAMES hiredis
    PATH ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (REDIS)
    message (STATUS "redis client ${REDIS} ${Color}ok${ColorReset}")
else ()
    message (STATUS "redis cliend NG")
endif()

find_library (SNAPPY NAMES snappy
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (SNAPPY)
    message (STATUS "snappy ${SNAPPY} ${Color}ok${ColorReset}")
else ()
    message (STATUS "snappy NG")
endif ()

find_library (GTEST NAMES gtest
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (GTEST)
    message (STATUS "gtest ${GTEST} ${Color}ok${ColorReset}")
else ()
    message (STATUS "gtest NG")
endif ()

find_library (GTEST_MAIN NAMES gtest_main
    PATHS ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (GTEST_MAIN)
    message (STATUS "gtest_main ${GTEST_MAIN} ${Color}ok${ColorReset}")
else ()
    message (STATUS "gtest_main NG")
endif ()

find_library (GLOG NAMES glog
    PATH ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (GLOG)
    message (STATUS "glog ${GLOG} ${Color}ok${ColorReset}")
else ()
    message (STATUS "glog NG")
endif()

find_library (GFLAGS NAMES gflags
    PATH ~/lib . ./lib ${CMAKE_CURRENT_BINARY_DIR}/lib)
if (GLOG)
    message (STATUS "gflags ${GFLAG} ${Color}ok${ColorReset}")
else ()
    message (STATUS "gflags NG")
endif()

if (BOOST_MPI AND BOOST_SYSTEM AND BOOST_THREAD AND BOOST_SERIALIZATION AND BOOST_FILESYSTEM AND LEVEL_DB AND REDIS AND SNAPPY AND GTEST AND GTEST_MAIN AND GLOG AND GFLAGS)
    message (STATUS "Library ${Color}ALL OK${ColorReset}")
else ()
    message (FATAL_ERROR "Library Dependency NG")
endif ()

### Include file ###
include_directories (. /usr/local/include include src test ${CMAKE_CURRENT_BINARY_DIR}/include )

### Compile own include files ###
add_library (scalelib STATIC
    include/cmake_dump.cc
    include/scalesim/com.hpp
    include/scalesim/com/event_communicator.hpp
    include/scalesim/com/gvt_communicator.hpp
    include/scalesim/com/mpi/collection.hpp
    include/scalesim/com/mpi/global_sync.hpp
    include/scalesim/com/mpi/mpi_runner.hpp
    include/scalesim/com/mpi/sender_receiver.hpp
    include/scalesim/logical_process.hpp
    include/scalesim/logical_process/logical_process.hpp
    include/scalesim/logical_process/process_scheduler.hpp
    include/scalesim/logical_process/queue.hpp
    include/scalesim/logical_process/store/berkeleydb_store.hpp
    include/scalesim/logical_process/store/leveldb_store.hpp
    include/scalesim/logical_process/store/redis_store.hpp
    include/scalesim/logical_process/store/store_base.hpp
    include/scalesim/simulation.hpp
    include/scalesim/simulation/application.hpp
    include/scalesim/simulation/runner.hpp
    include/scalesim/simulation/sim_obj.hpp
    include/scalesim/util.hpp
    include/scalesim/util/flags.hpp
    include/scalesim/util/message_counter.hpp
    include/scalesim/util/stopwatch.hpp
    include/scalesim/util/thread_manager.hpp
    include/scalesim/util/timestamp.hpp
    include/scalesim/util/type.hpp
    include/scalesim/util/version.hpp)

add_library (scale_testlib STATIC
    include/cmake_dump.cc
    test/test_app.hpp)

### TEST file ###
add_executable (TEST test/main.cc)
target_link_libraries (TEST
    scalelib scale_testlib ${BOOST_MPI} ${BOOST_SYSTEM} ${BOOST_THREAD} ${BOOST_SERIALIZATION} ${BOOST_FILESYSTEM} ${LEVEL_DB} ${SNAPPY} ${BERKELEY_DB} ${REDIS} ${GLOG} ${GFLAGS} ${GTEST} ${GTEST_MAIN})

### Traffic simulation ###
add_executable (TrafficSim
    ./src/trafficsim/main_trafficsim.cc)

target_link_libraries (TrafficSim
    scalelib ${BOOST_MPI} ${BOOST_SYSTEM} ${BOOST_THREAD} ${BOOST_SERIALIZATION} ${BOOST_FILESYSTEM} ${LEVEL_DB} ${SNAPPY} ${BERKELEY_DB} ${REDIS} ${GLOG} ${GFLAGS})
