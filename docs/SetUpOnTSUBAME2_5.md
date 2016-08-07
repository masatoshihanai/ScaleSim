# Set up for TSUBAME 2.5 Supercomputer
This is a document for setting up ScaleSim on TSUBAME 2.5 Supercomputer.

## Overview
1. Change MPI environment & C/C++ compiler
2. Change CMake version
3. Set up Boost Library with Boost.MPI
4. Set up Databases (LevelDB, Redis, Berkeley DB)
5. Set up gtest, gflags, glog
6. Copy CMakeLists.txt for TSUBAME

## 1. Change MPI environment & C/C++ compiler
In default, TSUBME 2.5 uses OpenMPI (1.6.5) with Intel compiler (i2013.1.046).
Thus, you have to change it to OpenMPI (1.6.5) with GCC compiler (version 5.x).
```
$ source set_ompi-1.6.5_g4.3.4.sh
$ source /usr/apps.sp3/nosupport/gsic/env/gcc-5.2.sh
```
Or add in your `.bashrc`
```
source set_ompi-1.6.5_g4.3.4.sh
source /usr/apps.sp3/nosupport/gsic/env/gcc-5.2.sh
```

## 2. Change CMake version
In default, CMake version is 2.6. **ScaleSim** needs CMake 3.x.
You can change the version as that. It is OK to add in your `.bashrc`.
```
$ source cmake-3.0.2.sh
```

## 3. Set up Boost Library with Boost.MPI
**ScaleSim** needs Boost 1.57.x or later with Boost.MPI.
Download the latest Boost and make a path to its top directory (`BOOST_HOME`).
In `.bashrc`, add
```
export BOOST_HOME="(your Boost folder)"
```
And then, move to the `BOOST_HOME`.
```
$ source ~/.bashrc
$ cd $BOOST_HOME
```
Finally, compile with **Boost.MPI**.
```
$ ./bootstrap
$ ./b2 --with-mpi
```

## 4. Set up Databases (LevelDB, Redis, Berkeley DB)
**ScaleSim** uses database for storing logs.
Download the latest databases and compile as that.

(LevelDB)
```
$ cd (your levelDB)
$ make
$ cp -r include/* ~/include
$ cp -r libleveldb.a ~/lib
```

(Berkeley DB)
```
$ cd (your BerkeleyDB)
$ ./dist/configure --enable-cxx --prefix=$HOME
$ make
$ make install
```

(Redis)
_UNDER CONSTRUCTION_

## 5. Set up gtest, gflags, glog, snappy
(gtest, gflags, glog)
```
$ cd (your gtest, gflags, glog folder)
$ mkdir mybuild && cd mybuild
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME ..
$ make install
```
(snappy)
```
$ cd (your snappy folder)
$ ./autogen.sh
$ ./configure --prefix=$HOME
$ make
$ make install
```
## 6. Copy CMakeLists.txt for TSUBAME
There is a CMakeLists.txt file for TSUBAME [`in conf/`](../conf/CMakeLists.txt.TSUBAME).
Replace an original CMakeLists.txt with the CMakeLists.txt.TSUBAME.
```
$ cd (ScaleSim folder)
$ mv CMakeLists.txt CMakeLists.txt.org
$ cp conf/CMakeLists.txt.TSUBAME CMakeLists.txt
```

## References
TSUBAME document
- (HOME) http://tsubame.gsic.titech.ac.jp/en/top
- (How to use unsupported latest software) http://tsubame.gsic.titech.ac.jp/labs (in Japanese)
- (How to change MPI & GCC environment) http://tsubame.gsic.titech.ac.jp/en/node/782

Boost document
- (HOME) http://www.boost.org
- (Getting stated with Boost.MPI) http://www.boost.org/doc/libs/release/doc/html/mpi/getting_started.html

Database documents
- LevelDB
  - (HOME) https://github.com/google/leveldb
  - (Docs) https://rawgit.com/google/leveldb/master/doc/index.html

- BerkeleyDB
  - (HOME) http://docs.oracle.com/cd/E17076_05/html/index.html
  - (Building for UNIX/POSIX) http://docs.oracle.com/cd/E17076_05/html/installation/build_unix_conf.html

gtest, gflags, glog documents
- (gtest HOME) https://github.com/google/googletest
- (gflags HOME) https://github.com/gflags/gflags
- (glog HOME) https://github.com/google/glog

CMAKE
- (HOME) https://cmake.org
