``mh5_hardware`` package
========================

This package follows the ``ros_control`` design model. It contains the highly specific hardware access functions needed for:

- configuring and communicating with the Dynamixel actuators used by the robot
- configuring and reading information from the on-board IMU unit
- (to-be) configuring and retrieving information from the Force Sensitive Resistors (FSRs) in the feet

.. doxygenclass:: mh5_hardware::MH5DynamixelInterface
    :no-link:
