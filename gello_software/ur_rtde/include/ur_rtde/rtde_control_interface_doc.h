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

static const char *__doc_ur_rtde_RTDEControlInterface =
R"doc(This class provides the interface to control the robot and to execute
robot movements. \note Currently the RTDEControlInterface, should not
be considered thread safe, since no measures (mutexes) are taken to
ensure that a function is done, before another would be executed. It
is up to the caller to provide protection using mutexes.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_disconnect =
R"doc(Returns:
    Can be used to disconnect from the robot. To reconnect you have to
    call the reconnect() function.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_endTeachMode = R"doc(Set robot back in normal position control mode after freedrive mode.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getAsyncOperationProgress =
R"doc(Reads progress information for asynchronous operations that supports
progress feedback (such as movePath). @retval <0 Indicates that no
async operation is running or that an async operation has finished.
The returned values of two consecutive async operations is never
equal. Normally the returned values are toggled between -1 and -2.
This allows the application to clearly detect the end of an operation
even if it is too short to see its start. That means, if the value
returned by this function is less than 0 and is different from that
last value returned by this function, then a new async operation has
finished. @retval 0 Indicates that an async operation has started -
progress 0 @retval >= 0 Indicates the progress of an async operation.
For example, if an operation has 3 steps, the progress ranges from 0 -
2. The progress value is updated, before a step is executed. When the
last step has been executed, the value will change to -1 to indicate
the end of the async operation. @deprecated The function is deprecated and only
here for backward compatibility. Use getAsyncOperationProgressEx() instead.)doc";


static const char *__doc_ur_rtde_RTDEControlInterface_getAsyncOperationProgressEx =
R"doc(Returns extended async operation progress information for asynchronous
operations that supports progress feedback (such as movePath).
@see AsyncOperationStatus documentation for a detailed description of the
returned status.)doc";


static const char *__doc_ur_rtde_RTDEControlInterface_getRobotStatus =
R"doc(Returns:
    Robot status Bits 0-3: Is power on | Is program running | Is teach
    button pressed | Is power button pressed
    There is a synchronization gap between the three interfaces RTDE Control
    RTDE Receive and Dashboard Client. RTDE Control and RTDE Receive open
    its own RTDE connection and so the internal state is not in sync. That
    means, if RTDE Control reports, that program is running, RTDE Receive may
    still return that program is not running. The update of the Dashboard
    Client even needs more time. That means, the dashboard client still
    returns program not running after some milliseconds have passed after
    RTDE Control already reports program running.
    \note If you work with RTDE control and receive interface and you need to
    read the robot status or program running state, then you should always
    use the getRobotStatus() function from RTDE Control if you need a status
    that is in sync with the program uploading or reuploading of this object.)doc";


static const char *__doc_ur_rtde_RTDEControlInterface_forceMode =
R"doc(Set robot to be controlled in force mode

Parameter ``task_frame``:
    A pose vector that defines the force frame relative to the base
    frame.

Parameter ``selection_vector``:
    A 6d vector of 0s and 1s. 1 means that the robot will be compliant
    in the corresponding axis of the task frame

Parameter ``wrench``:
    The forces/torques the robot will apply to its environment. The
    robot adjusts its position along/about compliant axis in order to
    achieve the specified force/torque. Values have no effect for non-
    compliant axes

Parameter ``type``:
    An integer [1;3] specifying how the robot interprets the force
    frame. 1: The force frame is transformed in a way such that its
    y-axis is aligned with a vector pointing from the robot tcp
    towards the origin of the force frame. 2: The force frame is not
    transformed. 3: The force frame is transformed in a way such that
    its x-axis is the projection of the robot tcp velocity vector onto
    the x-y plane of the force frame.

Parameter ``limits``:
    (Float) 6d vector. For compliant axes, these values are the
    maximum allowed tcp speed along/about the axis. For non-compliant
    axes, these values are the maximum allowed deviation along/about
    an axis between the actual tcp position and the one set by the
    program.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_forceModeSetDamping =
R"doc(Sets the damping parameter in force mode.

Parameter ``damping``:
    Between 0 and 1, default value is 0.005

A value of 1 is full damping, so the robot will decellerate quickly if
no force is present. A value of 0 is no damping, here the robot will
maintain the speed.

The value is stored until this function is called again. Call this
function before force mode is entered (otherwise default value will be
used).)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_forceModeSetGainScaling =
R"doc(Scales the gain in force mode.

Parameter ``scaling``:
    scaling parameter between 0 and 2, default is 1.

A value larger than 1 can make force mode unstable, e.g. in case of
collisions or pushing against hard surfaces.

The value is stored until this function is called again. Call this
function before force mode is entered (otherwise default value will be
used))doc";

static const char *__doc_ur_rtde_RTDEControlInterface_forceModeStop = R"doc(Resets the robot mode from force mode to normal operation.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getActualJointPositionsHistory =
R"doc(Returns the actual past angular positions of all joints.

This function returns the angular positions as reported by the function "get_actual_joint_positions()" which
indicates the number of controller time steps occurring before the current time step.

An exception is thrown if indexing goes beyond the buffer size.

Parameter ``steps``:
    The number of controller time steps required to go back. 0 corresponds to "get_actual_joint_positions()".

Returns:
    The joint angular position vector in rad : [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] that was
    actual at the provided number of steps before the current time step.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getForwardKinematics =
R"doc(Calculate the forward kinematic transformation (joint space -> tool
space) using the calibrated robot kinematics. If no joint position
vector is provided the current joint angles of the robot arm will be
used. If no tcp is provided the currently active tcp of the controller
will be used.

NOTICE! If you specify the tcp_offset you must also specify the q.

Parameter ``q``:
    joint position vector (Optional)

Parameter ``tcp_offset``:
    tcp offset pose (Optional)

Returns:
    the forward kinematic transformation as a pose)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getInverseKinematics =
R"doc(Calculate the inverse kinematic transformation (tool space ->
jointspace). If qnear is defined, the solution closest to qnear is
returned.Otherwise, the solution closest to the current joint
positions is returned. If no tcp is provided the currently active tcp
of the controller will be used.

Parameter ``x``:
    tool pose

Parameter ``qnear``:
    list of joint positions (Optional)

Parameter ``maxPositionError``:
    the maximum allowed positionerror (Optional)

Parameter ``maxOrientationError``:
    the maximum allowed orientationerror (Optional)

Returns:
    joint positions)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getJointTorques =
R"doc(Returns the torques of all joints

The torque on the joints, corrected by the torque needed to move the
robot itself (gravity, friction, etc.), returned as a vector of length
6.

Returns:
    The joint torque vector in Nm: [Base, Shoulder, Elbow, Wrist1,
    Wrist2, Wrist3])doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getStepTime =
R"doc(Returns the duration of the robot time step in seconds.

In every time step, the robot controller will receive measured joint
positions and velocities from the robot, and send desired joint
positions and velocities back to the robot. This happens with a
predetermined frequency, in regular intervals. This interval length is
the robot time step.

Returns:
    Duration of the robot step in seconds or 0 in case of an error)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getTCPOffset =
R"doc(Gets the active tcp offset, i.e. the transformation from the output
flange coordinate system to the TCP as a pose.

Returns:
    the TCP offset as a pose)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_getTargetWaypoint =
R"doc(Returns the target waypoint of the active move

This is different from the target tcp pose which returns the target
pose for each time step. The get_target_waypoint() returns the same
target pose for movel, movej, movep or movec during the motion. It
returns the target tcp pose, if none of the mentioned move functions
are running.

This method is useful for calculating relative movements where the
previous move command uses blends.

Returns:
    The desired waypoint TCP vector [X, Y, Z, Rx, Ry, Rz] or and empty
    vector in case of an error.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isConnected =
R"doc(Returns:
    Connection status for RTDE, useful for checking for lost
    connection.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isEmergencyStopped = R"doc()doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isJointsWithinSafetyLimits =
R"doc(Checks if the given joint position is reachable and within the current
safety limits of the robot. This check considers joint limits (if the
target pose is specified as joint positions), safety planes limits,
TCP orientation deviation limits and range of the robot. If a solution
is found when applying the inverse kinematics to the given target TCP
pose, this pose is considered reachable

Parameter ``q``:
    joint positions

Returns:
    a bool indicating if the joint positions are within the safety
    limits.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isPoseWithinSafetyLimits =
R"doc(Checks if the given pose is reachable and within the current safety
limits of the robot. It checks safety planes limits, TCP orientation
deviation limits and range of the robot. If a solution is found when
applying the inverse kinematics to the given target TCP pose, this
pose is considered reachable.

Parameter ``pose``:
    target pose

Returns:
    a bool indicating if the pose is within the safety limits.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isProgramRunning =
R"doc(Returns true if a program is running on the controller, otherwise it
returns false)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isProtectiveStopped = R"doc()doc";

static const char *__doc_ur_rtde_RTDEControlInterface_isSteady =
R"doc(Checks if robot is fully at rest.

True when the robot is fully at rest, and ready to accept higher
external forces and torques, such as from industrial screwdrivers.

Note: This function will always return false in modes other than the
standard position mode, e.g. false in force and teach mode.

Returns:
    True when the robot is fully at rest. Returns False otherwise.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_jogStart =
R"doc(Starts jogging with the given speed vector with respect to the given
feature. When jogging has started, it is possible to provide new speed
vectors by calling the jogStart() function over and over again. This
makes it possible to use a joystick or a 3D Space Navigator to provide
new speed vectors if the user moves the joystick or the Space
Navigator cap.

Parameter ``speed``:
    Speed vector for translation and rotation. Translation values are
    given in mm / s and rotation values in rad / s.

Parameter ``feature``:
    Configures to move to move with respect to base frame
    (FEATURE_BASE), tool frame (FEATURE_TOOL) or custom frame (FEATURE_CUSTOM)
    If the feature is FEATURE_CUSTOM then the custom_frame parameter needs to
    be a valid pose.

Parameter ``custom_frame``:
    The custom_frame given as pose if the selected feature
    is FEATURE_CUSTOM)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_jogStop = R"doc(Stops jogging that has been started start_jog)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_kickWatchdog =
R"doc(Kicks the watchdog safeguarding the communication. Normally you would
kick the watchdog in your control loop. Be sure to kick it as often as
specified by the minimum frequency of the watchdog.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveC =
R"doc(Move Circular: Move to position (circular in tool-space)

Parameter ``pose_via``:
    path point (note: only position is used)

Parameter ``pose_to``:
    target pose (note: only position is used in Fixed orientation
    mode).

Parameter ``speed``:
    tool speed [m/s]

Parameter ``acceleration``:
    tool acceleration [m/s^2]

Parameter ``blend``:
    blend radius [m]

Parameter ``mode``:
    0: Unconstrained mode. Interpolate orientation from current pose
    to target pose (pose_to) 1: Fixed mode. Keep orientation constant
    relative to the tangent of the circular arc (starting from current
    pose))doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveJ =
R"doc(Move to joint position (linear in joint-space)

Parameter ``q``:
    joint positions

Parameter ``speed``:
    joint speed of leading axis [rad/s]

Parameter ``acceleration``:
    joint acceleration of leading axis [rad/s^2]

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveJ_2 =
R"doc(Move to each joint position specified in a path

Parameter ``path``:
    with joint positions that includes acceleration, speed and blend
    for each position

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveJ_IK =
R"doc(Move to pose (linear in joint-space)

Parameter ``pose``:
    target pose

Parameter ``speed``:
    joint speed of leading axis [rad/s]

Parameter ``acceleration``:
    joint acceleration of leading axis [rad/s^2]

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveL =
R"doc(Move to position (linear in tool-space)

Parameter ``pose``:
    target pose

Parameter ``speed``:
    tool speed [m/s]

Parameter ``acceleration``:
    tool acceleration [m/s^2]

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveL_2 =
R"doc(Move to each pose specified in a path

Parameter ``path``:
    with tool poses that includes acceleration, speed and blend for
    each position

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveL_FK =
R"doc(Move to position (linear in tool-space)

Parameter ``q``:
    joint positions

Parameter ``speed``:
    tool speed [m/s]

Parameter ``acceleration``:
    tool acceleration [m/s^2]

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveP =
R"doc(Move Process: Blend circular (in tool-space) and move linear (in tool-
space) to position. Accelerates to and moves with constant tool speed
v.

Parameter ``pose``:
    target pose

Parameter ``speed``:
    tool speed [m/s]

Parameter ``acceleration``:
    tool acceleration [m/s^2]

Parameter ``blend``:
    blend radius [m])doc";

static const char *__doc_ur_rtde_RTDEControlInterface_movePath =
R"doc(Move to each waypoint specified in the given path

Parameter ``path``:
    The path with waypoints @param

Parameter ``async``:
    a bool specifying if the move command should be asynchronous. If
    async is true it is possible to stop a move command using either
    the stopJ or stopL function. Default is false, this means the
    function will block until the movement has completed.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_moveUntilContact =
R"doc(Move the robot until contact, with specified speed and contact
detection direction.

The robot will automatically retract to the initial point of contact.

Parameter ``xd``:
    tool speed [m/s] (spatial vector)

Parameter ``direction``:
    List of six floats. The first three elements are interpreted as a
    3D vector (in the robot base coordinate system) giving the
    direction in which contacts should be detected. If all elements of
    the list are zero, contacts from all directions are considered.
    You can also set direction=get_target_tcp_speed() in which case it
    will detect contacts in the direction of the TCP movement.

Parameter ``acceleration``:
    tool position acceleration [m/s^2]

Returns:
    True once the robot is in contact.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_poseTrans =
R"doc(Pose transformation to move with respect to a tool or w.r.t. a custom
feature/frame The first argument, p_from, is used to transform the
second argument, p_from_to, and the result is then returned. This
means that the result is the resulting pose, when starting at the
coordinate system of p_from, and then in that coordinate system moving
p_from_to. This function can be seen in two different views. Either
the function transforms, that is translates and rotates, p_from_to by
the parameters of p_from. Or the function is used to get the resulting
pose, when first making a move of p_from and then from there, a move
of p_from_to. If the poses were regarded as transformation matrices,
it would look like: @verbatim T_world->to = T_world->from * T_from->to
T_x->to = T_x->from * T_from->to @endverbatim

Parameter ``p_from``:
    starting pose (spatial vector)

Parameter ``p_from_to``:
    pose change relative to starting pose (spatial vector)

Returns:
    resulting pose (spatial vector))doc";

static const char *__doc_ur_rtde_RTDEControlInterface_reconnect =
R"doc(Returns:
    Can be used to reconnect to the robot after a lost connection.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_reuploadScript =
R"doc(In the event of an error, this function can be used to resume
operation by reuploading the RTDE control script. This will only
happen if a script is not already running on the controller.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_sendCustomScript =
R"doc(Send a custom ur script to the controller The function enables sending
of short scripts which was defined inline within source code. So you
can write code like this:

```
const std::string inline_script =
"def script_test():\n"
"\tdef test():\n"
"textmsg(\"test1\")\n"
"textmsg(\"test2\")\n"
"\tend\n"
"\twrite_output_integer_register(0, 1)\n"
"\ttest()\n"
"\ttest()\n"
"\twrite_output_integer_register(0, 2)\n"
"end\n"
"run program\n";
bool result = rtde_c.sendCustomScript(inline_script);
```

Returns:
    Returns true if the script has been executed successfully and
    false on timeout)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_sendCustomScriptFile =
R"doc(Send a custom ur script file to the controller

Parameter ``file_path``:
    the file path to the custom ur script file)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_sendCustomScriptFunction =
R"doc(Send a custom ur script to the controller

Parameter ``function_name``:
    specify a name for the custom script function

Parameter ``script``:
    the custom ur script to be sent to the controller specified as a
    string, each line must be terminated with a newline. The code will
    automatically be indented with one tab to fit with the function
    body.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_servoC =
R"doc(Servo to position (circular in tool-space). Accelerates to and moves
with constant tool speed v.

Parameter ``pose``:
    target pose

Parameter ``speed``:
    tool speed [m/s]

Parameter ``acceleration``:
    tool acceleration [m/s^2]

Parameter ``blend``:
    blend radius (of target pose) [m])doc";

static const char *__doc_ur_rtde_RTDEControlInterface_servoJ =
R"doc(Servo to position (linear in joint-space)

Parameter ``q``:
    joint positions [rad]

Parameter ``speed``:
    NOT used in current version

Parameter ``acceleration``:
    NOT used in current version

Parameter ``time``:
    time where the command is controlling the robot. The function is
    blocking for time t [S]

Parameter ``lookahead_time``:
    time [S], range [0.03,0.2] smoothens the trajectory with this
    lookahead time

Parameter ``gain``:
    proportional gain for following target position, range [100,2000])doc";

static const char *__doc_ur_rtde_RTDEControlInterface_servoL =
R"doc(Servo to position (linear in tool-space)

Parameter ``pose``:
    target pose

Parameter ``speed``:
    NOT used in current version

Parameter ``acceleration``:
    NOT used in current version

Parameter ``time``:
    time where the command is controlling the robot. The function is
    blocking for time t [S]

Parameter ``lookahead_time``:
    time [S], range [0.03,0.2] smoothens the trajectory with this
    lookahead time

Parameter ``gain``:
    proportional gain for following target position, range [100,2000])doc";

static const char *__doc_ur_rtde_RTDEControlInterface_servoStop =
R"doc(Stop servo mode and decelerate the robot.

Parameter ``a``:
    rate of deceleration of the tool [m/s^2])doc";

static const char *__doc_ur_rtde_RTDEControlInterface_setCustomScriptFile =
R"doc(Assign a custom script file that will be sent to device as the main
control script. Setting an empty file_name will disable the custom
script loading This eases debugging when modifying the control script
because it does not require to recompile the whole library)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_setPayload =
R"doc(Set payload

Parameter ``mass``:
    Mass in kilograms

Parameter ``cog``:
    Center of Gravity, a vector [CoGx, CoGy, CoGz] specifying the
    displacement (in meters) from the toolmount. If not specified the
    current CoG will be used.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_setTcp =
R"doc(Sets the active tcp offset, i.e. the transformation from the output
flange coordinate system to the TCP as a pose.

Parameter ``tcp_offset``:
    A pose describing the transformation of the tcp offset.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_setWatchdog =
R"doc(Enable a watchdog for the communication with a specified minimum
frequency for which an input update is expected to arrive. The
watchdog is useful for safety critical realtime applications eg.
servoing. The default action taken is to shutdown the control, if the
watchdog is not kicked with the minimum frequency.

Preferably you would call this function right after the
RTDEControlInterface has been constructed.

Parameter ``min_frequency``:
    The minimum frequency an input update is expected to arrive
    defaults to 10Hz.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_speedJ =
R"doc(Joint speed - Accelerate linearly in joint space and continue with
constant joint speed

Parameter ``qd``:
    joint speeds [rad/s]

Parameter ``acceleration``:
    joint acceleration [rad/s^2] (of leading axis)

Parameter ``time``:
    time [s] before the function returns (optional))doc";

static const char *__doc_ur_rtde_RTDEControlInterface_speedL =
R"doc(Tool speed - Accelerate linearly in Cartesian space and continue with
constant tool speed. The time t is optional;

Parameter ``xd``:
    tool speed [m/s] (spatial vector)

Parameter ``acceleration``:
    tool position acceleration [m/s^2]

Parameter ``time``:
    time [s] before the function returns (optional))doc";

static const char *__doc_ur_rtde_RTDEControlInterface_speedStop =
R"doc(Stop speed mode and decelerate the robot.

Parameter ``a``:
    rate of deceleration of the tool [m/s^2] if using speedL, for
    speedJ its [rad/s^2] and rate of deceleration of leading axis.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_stopJ =
R"doc(Stop (linear in joint space) - decelerate joint speeds to zero

Parameter ``a``:
    joint acceleration [rad/s^2] (rate of deceleration of the leading
    axis).)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_stopL =
R"doc(Stop (linear in tool space) - decelerate tool speed to zero

Parameter ``a``:
    tool acceleration [m/s^2] (rate of deceleration of the tool))doc";

static const char *__doc_ur_rtde_RTDEControlInterface_stopScript = R"doc(This function will terminate the script on controller.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_teachMode =
R"doc(Set robot in freedrive mode. In this mode the robot can be moved
around by hand in the same way as by pressing the "freedrive" button.
The robot will not be able to follow a trajectory (eg. a movej) in
this mode.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_toolContact =
R"doc(Detects when a contact between the tool and an object happens.

Parameter ``direction``:
    The first three elements are interpreted as a 3D vector (in the
    robot base coordinate system) giving the direction in which
    contacts should be detected. If all elements of the list are zero,
    contacts from all directions are considered.

Returns:
    The returned value is the number of time steps back to just before
    the contact have started. A value larger than 0 means that a
    contact is detected. A value of 0 means no contact.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_triggerProtectiveStop =
R"doc(Triggers a protective stop on the robot. Can be used for testing and
debugging.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_waitForProgramRunning =
R"doc(This function waits until the script program is running. If the
program is not running after a certain amount of time, the function
tries to resend the script. If the script is not running after the
timeout time, an exception is thrown.)doc";

static const char *__doc_ur_rtde_RTDEControlInterface_zeroFtSensor =
R"doc(Zeroes the TCP force/torque measurement from the builtin force/torque
sensor by subtracting the current measurement from the subsequent.)doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

