#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/francesco/Downloads/isaac-sim-standalone@2023.1.1-rc.8+2023.1.688.573e0291.tc.linux-x86_64.release/exts/omni.isaac.ros2_bridge/humble/lib

/home/francesco/Downloads/isaac-sim-standalone@2023.1.1-rc.8+2023.1.688.573e0291.tc.linux-x86_64.release/isaac-sim.selector.sh
