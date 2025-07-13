# pez_comms (ROS Noetic)

ROS 1 version of the communication node.  It shares the same YAML
configuration format as the ROS 2 package but uses `rospy` under the
hood.  The node is launched with `comms.launch.xml` and reads the
serial modem parameters and packet layout from the YAML files in
`config/`.

The Noetic implementation is intended to run as a BlueOS extension so
that a BlueROV2 can communicate with a Humble host via the acoustic
modem.
