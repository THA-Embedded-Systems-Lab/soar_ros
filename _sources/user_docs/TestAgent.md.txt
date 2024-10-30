# Soar Test Agent for Development

The `test_example` program is a minimum working example of a Soar agent
integrated in ROS2 for communicating. It is used for testing but also as an
example on how to set up the project structure.

The tests can be executed either via `colcon test` or
`launch_test test/test_launch.py`.

**Known Bug**: Soar does not exit cleanly after unit tests and the `test_example`
program must be terminated manually.
