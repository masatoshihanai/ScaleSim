# ScaleSim

ScaleSim is a large-scale distributed and parallel discrete event simulator,
based on optimistic PDES (Parallel Discrete Event Simulation).

ScaleSim provides **_Exact-Differential_** execution, proposed in

- *"Exact-Differential Large-Scale Traffic Simulation" (SIGSIM PADS 2015)*
- *"Towards Large-Scale What-If Traffic Simulation with Exact-Differential Simulation" (WSC 2015)*

## Requirement
- `MPI (Open MPI)`
- `boost`, `boost_mpi`, `boost_thread`, `boost_system`, `boost_serialization`, `boost_filesystem`,
- `leveldb`, `snappy`,
- `gtest`, `glog`, `gflags`,

#### Tested version
- `Open MPI` 1.8.8 (MPI 3.0), 1.10.0 (MPI 3.0)
- `boost` 1.57.0, 1.58.0
- `leveldb` 1.18, `snappy` 1.1.3
- `gtest` 1.7.0, `glog` 0.3.3, `gflags` 2.1.2

## Build
Make a build directory
```
$ mkdir build
$ cd build
```

Configure and compile (require c++11)
```
$ cmake ..
$ make
```

## Run Example
Single Simulation
```
$ mpirun -np 4 ./build/TrafficSim \
                 ./traffic/ring/part/graph.part.4 \
                 ./traffic/ring/rd.sim.csv \
                 ./traffic/ring/trip.csv
```

### Exact-Differential Simulation

You should define:
- `--sim_id` : simulation task ID. In repeating execution, the simulator reads same ID inputs.
- `--store_dir` or `--store_ip` : directory path or IP address to store results.

For initial execution, use `--diff_init` option.
```
$ mpirun -np 4 ./build/TrafficSim --sim_id=999 --store_dir="./tmp" --diff_init \
                 ./traffic/ring/part/graph.part.4 \
                 ./traffic/ring/part/rd.sim.csv \
                 ./traffic/ring/trip.csv
```

For repeat execution, use `--diff_repeat` option.
```    
$ mpirun -np 4 ./build/TrafficSim --sim_id=999 --store_dir="./tmp" --diff_repeat \   
                 ./traffic/ring/part/graph.part.4 \
                 ./traffic/ring/rd.sim.csv \
                 ./traffic/ring/add_ev.csv  
```

### Test

    $ ./build/Test

You can filter test types with gtest_filter

    $ ./build/Test --gtest_filter=*small　  # Small test
    $ ./build/Test --gtest_filter=*meduim   # Medium test
    $ ./build/Test --gtest_filter=*large    # Large test
