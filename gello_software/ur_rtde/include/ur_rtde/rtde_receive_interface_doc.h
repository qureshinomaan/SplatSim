/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by pybind11_mkdoc.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif


static const char *__doc_ur_rtde_RTDEReceiveInterface = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_disconnect =
R"doc(Returns:
    Can be used to disconnect from the robot. To reconnect you have to
    call the reconnect() function.)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualCurrent =
R"doc(Returns:
    Actual joint currents)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualDigitalInputBits =
R"doc(Returns:
    Current state of the digital inputs. 0-7: Standard, 8-15:
    Configurable, 16-17: Tool)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualDigitalOutputBits =
R"doc(Returns:
    Current state of the digital outputs. 0-7: Standard, 8-15:
    Configurable, 16-17: Tool)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualExecutionTime =
R"doc(Returns:
    Controller real-time thread execution time)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualJointVoltage =
R"doc(Returns:
    Actual joint voltages)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualMainVoltage =
R"doc(Returns:
    Safety Control Board: Main voltage)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualMomentum =
R"doc(Returns:
    Norm of Cartesian linear momentum)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualQ =
R"doc(Returns:
    Actual joint positions)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualQd =
R"doc(Returns:
    Actual joint velocities)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualRobotCurrent =
R"doc(Returns:
    Safety Control Board: Robot current)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualRobotVoltage =
R"doc(Returns:
    Safety Control Board: Robot voltage (48V))doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualTCPForce =
R"doc(Returns:
    Generalized forces in the TCP)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualTCPPose =
R"doc(Returns:
    Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where
    rx, ry and rz is a rotation vector representation of the tool
    orientation)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualTCPSpeed =
R"doc(Returns:
    Actual speed of the tool given in Cartesian coordinates)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualToolAccelerometer =
R"doc(Returns:
    Tool x, y and z accelerometer values)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getDigitalOutState =
R"doc(Test if a digital output is set 'high' or 'low' the range is 0-7:
Standard, 8-15: Configurable, 16-17: Tool

Parameter ``output_id``:
    the id of the digital output to test

Returns:
    a bool indicating the state of the digital output)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getJointControlOutput =
R"doc(Returns:
    Joint control currents)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getJointMode =
R"doc(Returns:
    Joint control modes)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getJointTemperatures =
R"doc(Returns:
    Temperature of each joint in degrees Celsius)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getOutputDoubleRegister =
R"doc(Get the specified output double register in either lower range [18-22]
or upper range [42-46].

Parameter ``output_id``:
    the id of the register to read, current supported range is:
    [18-22] or [42-46], this can be adjusted by changing the
    RTDEReceiveInterface output recipes and by using the
    use_upper_range_registers constructor flag to switch between lower
    and upper range.

Returns:
    a double from the specified output register)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getOutputIntRegister =
R"doc(Get the specified output integer register in either lower range
[18-22] or upper range [42-46].

Parameter ``output_id``:
    the id of the register to read, current supported range is:
    [18-22] or [42-46], this can be adjusted by changing the
    RTDEReceiveInterface output recipes and by using the
    use_upper_range_registers constructor flag to switch between lower
    and upper range.

Returns:
    an integer from the specified output register)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getRobotMode =
R"doc(Returns:
    Robot mode -1 = ROBOT_MODE_NO_CONTROLLER 0 =
    ROBOT_MODE_DISCONNECTED 1 = ROBOT_MODE_CONFIRM_SAFETY 2 =
    ROBOT_MODE_BOOTING 3 = ROBOT_MODE_POWER_OFF 4 =
    ROBOT_MODE_POWER_ON 5 = ROBOT_MODE_IDLE 6 = ROBOT_MODE_BACKDRIVE 7
    = ROBOT_MODE_RUNNING 8 = ROBOT_MODE_UPDATING_FIRMWARE)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getRobotStatus =
R"doc(Returns:
    Robot status Bits 0-3: Is power on | Is program running | Is teach
    button pressed | Is power button pressed)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getRuntimeState =
R"doc(Returns:
    Program state)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getSafetyMode =
R"doc(Returns:
    Safety mode)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getSafetyStatusBits =
R"doc(Returns:
    Safety status bits Bits 0-10: Is normal mode | Is reduced mode |
    Is protective stopped | Is recovery mode | Is safeguard stopped |
    Is system emergency stopped | Is robot emergency stopped | Is
    emergency stopped | Is violation | Is fault | Is stopped due to
    safety)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getSpeedScaling =
R"doc(Returns:
    Speed scaling of the trajectory limiter)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getSpeedScalingCombined =
R"doc(Get the combined speed scaling The combined speed scaling is the speed
scaling resulting from multiplying the speed scaling with the target
speed fraction. The combined speed scaling takes the runtime_state of
the controller into account. If eg. a motion is paused on the teach
pendant, and later continued, the speed scaling will be ramped up from
zero and return to speed_scaling * target_speed_fraction when the
runtime_state is RUNNING again.

This is useful for scaling trajectories with the slider speed scaling
currently set on the teach pendant.

Returns:
    the actual combined speed scaling)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogInput0 =
R"doc(Returns:
    Standard analog input 0 [A or V])doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogInput1 =
R"doc(Returns:
    Standard analog input 1 [A or V])doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogOutput0 =
R"doc(Returns:
    Standard analog output 0 [A or V])doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogOutput1 =
R"doc(Returns:
    Standard analog output 1 [A or V])doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetCurrent =
R"doc(Returns:
    Target joint currents)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetMoment =
R"doc(Returns:
    Target joint moments (torques))doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetQ =
R"doc(Returns:
    Target joint positions)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetQd =
R"doc(Returns:
    Target joint velocities)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetQdd =
R"doc(Returns:
    Target joint accelerations)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetSpeedFraction =
R"doc(Returns:
    Target speed fraction)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetTCPPose =
R"doc(Returns:
    Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where
    rx, ry and rz is a rotation vector representation of the tool
    orientation)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetTCPSpeed =
R"doc(Returns:
    Target speed of the tool given in Cartesian coordinates)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTimestamp =
R"doc(Returns:
    Time elapsed since the controller was started [s])doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_isConnected =
R"doc(Returns:
    Connection status for RTDE, useful for checking for lost
    connection.)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_isEmergencyStopped =
R"doc(Returns:
    a bool indicating if the robot is in 'Emergency stop')doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_isProtectiveStopped =
R"doc(Returns:
    a bool indicating if the robot is in 'Protective stop')doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_reconnect =
R"doc(Returns:
    Can be used to reconnect to the robot after a lost connection.)doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

