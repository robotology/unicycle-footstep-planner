# unicycle-footstep-planner
This is a library aimed at generating walking trajectories, adopting the model of a unicycle under the hood. Some more details on the theory and the implementation will be coming soon.

## Dependencies

- **[``iDynTree``](https://github.com/robotology/idyntree)**, version 0.9.2

## Build the library and the application
### Linux/macOs
```
git clone https://github.com/robotology/unicycle-footstep-planner
cd unicycle-footstep-planner
mkdir build && cd build
cmake ..
make
[sudo] make install
```
Notice: ``sudo`` is not necessary if you specify the ``CMAKE_INSTALL_PREFIX``. In this case it is necessary to add in the ``.bashrc`` the following lines:
### Linux
```
export UnicyclePlanner_DIR=/path/where/you/installed/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$UnicyclePlanner_DIR/lib
```

### macOs
```
export UnicyclePlanner_DIR=/path/where/you/installed/
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH:$UnicyclePlanner_DIR/lib
```
## Basic Idea & Implementation
See https://github.com/robotology/unicycle-footstep-planner/issues/7
