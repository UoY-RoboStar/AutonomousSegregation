This workspace contains a partial ROS2 Jazzy implementation of the CacheCons model based on the automatically generated C code.

It includes two ROS packages: cachecons_launchers, cachecons_interface and cachecons_controller:
- cachecons_launchers provides a launch file to set up a simulation with two instances of the turtlebot3;
- cachecons_itnerface provides message definitions based on the types used in the model;
- cachecons_controller provides a node and auxiliary classes (type definitions and state machine behaviour) that implements the statemachine ObstacleAvoidance.