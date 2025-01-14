#pragma once
#ifndef ROBOTIQ_GRIPPER_H
#define ROBOTIQ_GRIPPER_H

#include <ur_rtde/rtde_export.h>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <stdexcept>
#include <memory>
#include <mutex>
#include <string>

namespace ur_rtde
{
/**
 * This exception is thrown if the gripper is in a wrong state to perform
 * the requested operation.
 * If the robot is in emergency stop state, then reading variables from the
 * gripper only returns a questionmark instead of the requested value.
 * In this case, the functions for reading values from the robot will throw
 * this exception
 */
class GripperStateException : public std::runtime_error
{
public:
	using std::runtime_error::runtime_error;
};

/**
 * C++ driver for Robot IQ grippers
 * Communicates with the gripper directly, via socket with string commands,
 * leveraging string names for variables.
 *
 * - WRITE VARIABLES (CAN ALSO READ):
 *   - ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
 *   - GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
 *   - ATR = 'ATR'  # atr : auto-release (emergency slow move)
 *   - ARD = 'ARD'  # ard : auto-release direction (open(1) or close(0) during auto-release)
 *   - FOR = 'FOR'  # for : force (0-255)
 *   - SPE = 'SPE'  # spe : speed (0-255)
 *   - POS = 'POS'  # pos : position (0-255), 0 = open
 * - READ VARIABLES
 *   - STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
 *   - PRE = 'PRE'  # position request (echo of last commanded position)
 *   - OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
 *   - FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)
 */
class RobotiqGripper
{
 public:
  /**
   * Gripper status reported by the gripper.
   * The integer values have to match what the gripper sends.
   */
  enum eStatus
  {
    RESET = 0,       //!< RESET
    ACTIVATING = 1,  //!< ACTIVATING
    // UNUSED = 2  # This value is currently not used by the gripper firmware
    ACTIVE = 3,  //!< ACTIVE
  };

  /**
   *  Object status reported by the gripper.
   *  The integer values have to match what the gripper sends.
   */
  enum eObjectStatus
  {
    MOVING = 0,                ///< gripper is opening or closing
    STOPPED_OUTER_OBJECT = 1,  ///< outer object detected while opening the gripper
    STOPPED_INNER_OBJECT = 2,  ///< inner object detected while closing the gripper
    AT_DEST = 3,               ///< requested target position reached - no object detected
  };

  /**
   * Connection status
   */
  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

  /**
   * For synchronous or asynchronous moves
   */
  enum eMoveMode
  {
    START_MOVE,    //!< returns immediately after move started
    WAIT_FINISHED  //!< returns if the move finished, that means if an object is gripped or if requested target position
                   //!< is reached
  };

  /**
   * Unit identifiers for the configuration of the units for position,
   * speed and flow.
   */
  enum eUnit
  {
    UNIT_DEVICE,      ///< device unit in the range 0 - 255 - 255 meansd fully closed and 0 means fully open
    UNIT_NORMALIZED,  ///< normalized value in the range 0.0 - 1.0, for position 0.0 means fully closed and 1.0 means
                      ///< fully open
    UNIT_PERCENT,  ///< percent in the range from 0 - 100 % for position 0% means fully closed and 100% means fully open
    UNIT_MM        ///< position value in mm - only for position values
  };

  /**
   * Identifier for move parameters
   */
  enum eMoveParameter
  {
    POSITION,
    SPEED,
    FORCE
  };

  /**
   * Position identifiers
   */
  enum ePostionId
  {
    OPEN = 0,
    CLOSE = 1,
  };

  /**
   * Fault code set in FLT
   */
  enum eFaultCode
  {
    NO_FAULT = 0x00,  ///< No fault (solid blue LED)

    // Priority faults (solid blue LED)
    FAULT_ACTION_DELAYED =
        0x05,  ///< Action delayed; the activation (re-activation) must be completed prior to perform the action.
    FAULT_ACTIVATION_BIT = 0x07,  ///< The activation bit must be set prior to performing the action.

    // Minor faults (solid red LED)
    FAULT_TEMPERATURE =
        0x08,  ///< Maximum operating temperature exceeded (≥ 85 °C internally); let cool down (below 80 °C).
    FAULT_COMM = 0x09,  ///< No communication during at least 1 second.

    // Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit (rACT) needed).
    FAULT_UNDER_VOLTAGE = 0x0A,         ///< Under minimum operating voltage.
    FAULT_EMCY_RELEASE_ACTIVE = 0x0B,   ///< Automatic release in progress.
    FAULT_INTERNAL = 0x0C,              ///< Internal fault, contact support@robotiq.com
    FAULT_ACTIVATION = 0x0D,            ///< Activation fault, verify that no interference or other error occurred.
    FAULT_OVERCURRENT = 0x0E,           ///< Overcurrent triggered.
    FAULT_EMCY_RELEASE_FINISHED = 0x0F  ///< Automatic release completed.
  };

  /**
   * Constructor - creates a RobotiqGripper object with the given Hostname/IP
   * and Port.
   * @param Hostname The hostname or ip address to use for connection
   * @param Port The port to use for connection
   * @param verbose Prints additional debug information if true
   */
  RTDE_EXPORT RobotiqGripper(const std::string& Hostname, int Port = 63352, bool verbose = false);

  /**
   * Connects to the gripper server with the given millisecond timeout
   */
  RTDE_EXPORT void connect(uint32_t timeout_ms = 2000);

  /**
   * Disconnects from the gripper server
   */
  RTDE_EXPORT void disconnect();

  /**
   * Returns true if connected
   */
  RTDE_EXPORT bool isConnected() const;

  /**
   * Resets the activation flag in the gripper, and sets it back to one,
   * clearing previous fault flags.
   * This is required after an emergency stop or if you just powered on
   * your robot.
   * \param auto_calibrate: Whether to calibrate the minimum and maximum
   * positions based on actual motion.
   */
  RTDE_EXPORT void activate(bool auto_calibrate = false);

  /**
   * Attempts to calibrate the open and closed positions, by closing
   * and opening the gripper.
   * The function returns if calibration has been finished.
   * \param[in] Speed Optional speed parameter. If the speed parameter is
   * less than 0, then is ignored and the calibration move is executed with
   * default calibration speed (normally 1/4 of max speed).
   */
  RTDE_EXPORT void autoCalibrate(float Speed = -1.0);

  /**
   * Returns whether the gripper is active.
   * If the function returns false, you need to activate the gripper via the
   * activate() function.
   * \see activate()
   */
  RTDE_EXPORT bool isActive();

  /**
   * Returns the fully open position in the configured position unit.
   * If you work with UNIT_DEVICE, then the open position is the position
   * value near 0. If you work with any other unit, then the open position
   * is the bigger value (that means 1.0 for UNIT_NORMALIZED, 100 for UNIT_PERCENT
   * or the opening in mm for UNIT_MM).
   */
  RTDE_EXPORT float getOpenPosition() const;

  /**
   * Returns what is considered the closed position for gripper in the
   * configured position unit.
   * If you work with UNIT_DEVICE, then the closed position is the position
   * value near 255. If you work with any other unit, then the closed position
   * is always 0. That means the position value defines the opening of the
   * gripper.
   */
  RTDE_EXPORT float getClosedPosition() const;

  /**
   * Returns the current position as returned by the physical hardware
   * in the configured position unit.
   */
  RTDE_EXPORT float getCurrentPosition();

  /**
   * Returns whether the current position is considered as being fully open.
   */
  RTDE_EXPORT bool isOpen();

  /**
   * Returns whether the current position is considered as being fully closed.
   */
  RTDE_EXPORT bool isClosed();

  /**
   * \brief Sends command to start moving towards the given position,
   * with the specified speed and force.
   * If the Speed and Force parameters are -1.0, then they are ignored and
   * the pre configured speed and force parameters set via setSpeed() and
   * setForce() function are used. So this gives you the option to pass
   * in the speed parameter each time or to use preset speed and
   * force parameters.
   * \param Position: Position to move to [getClosedPosition() to getOpenPosition()]
   * \param Speed: Speed to move at [min_speed, max_speed]
   * \param Force: Force to use [min_force, max_force]
   * \param MoveMode: START_MOVE - starts the move and returns immediately
   *        WAIT_FINISHED - waits until the move has finished
   * \return: Returns the object detection status.
   */
  RTDE_EXPORT int move(float Position, float Speed = -1.0, float Force = -1.0, eMoveMode MoveMode = START_MOVE);

  /**
   * Moves the gripper to its fully open position.
   * \see See move() function for a detailed description of all other parameters
   */
  RTDE_EXPORT int open(float Speed = -1.0, float Force = -1.0, eMoveMode MoveMode = START_MOVE);

  /**
   * Moves the gripper to its fully closed position.
   * \see See move() function for a detailed description of all other parameters
   */
  RTDE_EXPORT int close(float Speed = -1.0, float Force = -1.0, eMoveMode MoveMode = START_MOVE);

  /**
   * The emergency release is meant to disengage the gripper after an emergency
   * stop of the robot. The emergency open is not intended to be used
   * under normal operating conditions.
   * \param Direction OPEN - moves to fully open position, CLOSE - moves to
   * \param MoveMode WAIT_FINISHED - waits until emergency release has
   *        finished
   *        START_MOVE - returns as soon as the emergency release has started
   *        Use faultStatus() function o check when emergency release has
   *        finished. (faultStatus() == FAULT_EMCY_RELEASE_FINISHED)
   * 	fully close position
   */
  RTDE_EXPORT void emergencyRelease(ePostionId Direction, eMoveMode MoveMode = WAIT_FINISHED);

  /**
   * Returns the current fault status code.
   * \see eFaultCode
   */
  RTDE_EXPORT int faultStatus();

  /**
   * Set the units to use for passing position, speed and force values to the
   * gripper functions and for reading back values like current position.
   * The default unit for position is UNIT_NORM (0.0 - 1.0). That means gripper
   * closed is 0.0 and gripper fully open is 1.0
   * The default unit for speed and force is also UNIT_NORM (0.0 - 1.0).
   * That means 1.0 means maximum force and maximum speed and 0.5 means half
   * speed and half force.
   * \see eUnit
   */
  RTDE_EXPORT void setUnit(eMoveParameter Param, eUnit Unit);

  /**
   * Configure the position range of your gripper.
   * If you would like to use the unit UNIT_MM for position values, then
   * you need to properly configure the position range of your gripper
   * for proper value conversion.
   * \param[in] Range The position range in mm from the device position 0 to
   *            the device position 255
   */
  RTDE_EXPORT void setPositionRange_mm(int Range);

  /**
   * Sets the speed to use for future move commands in the configured
   * speed unit.
   * \return Returns the adjusted speed value.
   * \see move()
   */
  RTDE_EXPORT float setSpeed(float Speed);

  /**
   * Sets the force for future move commands in the configured
   * force unit
   * \return Returns the adjusted force value
   * \see move()
   */
  RTDE_EXPORT float setForce(float Force);

  /**
   * Returns the current object detection status if a move is active.
   * Use this function for polling the state.
   */
  RTDE_EXPORT eObjectStatus objectDetectionStatus();

  /**
   * Call this function after a move command to wait for completion of the
   * commanded move.
   */
  RTDE_EXPORT eObjectStatus waitForMotionComplete();

  /**
   * Sends the appropriate command via socket to set the value of n variables,
   * and waits for its 'ack' response.
   * \param Vars Dictionary of variables to set (variable_name, value).
   * \return: True on successful reception of ack, false if no ack was
   * 		received, indicating the set may not have been effective.
   */
  RTDE_EXPORT bool setVars(const std::vector<std::pair<std::string, int>> Vars);

  /**
   * Sends the appropriate command via socket to set the value of a variable,
   * and waits for its 'ack' response.
   * \param Var: Variable to set.
   * \param Value: Value to set for the variable.
   * \return: True on successful reception of ack, false if no ack was received,
   * indicating the set may not have been effective.
   */
  RTDE_EXPORT bool setVar(const std::string& Var, int Value);

  /**
   * Sends the appropriate command to retrieve the value of a variable from
   * the gripper, blocking until the response is received or the socket times out.
   * \param var: Name of the variable to retrieve.
   * \return: Value of the variable as integer.
   */
  RTDE_EXPORT int getVar(const std::string& var);

  /**
   * This function enables the reading of a number of variables into
   * a vector of values:
   * \code
   * std::vector<std::string> Vars{"STA", "OBJ", "ACT", "POS"};
   * auto Result = Gripper.getVars(Vars);
   * for (int i = 0; i < Vars.size(); ++i)
   * {
   *    std::cout << Vars[i] << ": " << Result[i] << std::endl;
   * }
   * \endcode
   */
  RTDE_EXPORT std::vector<int> getVars(const std::vector<std::string>& Vars);

  /**
   * Returns the native positions range in device units.
   * The native position range is properly initialized after an auto calibration.
   * \param[out] MinPosition Returns the detected minimum position
   * \param[out] MaxPosition Returns the detected maximum position
   */
  RTDE_EXPORT void getNativePositionRange(int& MinPosition, int& MaxPosition);

   /**
   * Sets the native position range in device units.
   * Normally the native position range is properly initialized after an
   * auto calibration. If you would like to avoid a calibration move and
   * if you have previously determined or calculated the naticve position range
   * then you can set it via this function.
   * \param[in] MinPosition Returns the detected minimum position
   * \param[in] MaxPosition Returns the detected maximum position
   */
  RTDE_EXPORT void setNativePositionRange(int MinPosition, int MaxPosition);

private:
  /**
   * Print all variables for debugging
   */
  void dumpVars();

  /**
   * Receive function to receive data from the gripper server
   */
  std::string receive();

  /**
   * Send function to send data to the gripper
   */
  void send(const std::string& str);

  /**
   * Resets the gripper.
   */
  void reset();

  /**
   * Returns the minimum position the gripper can reach
   */
  float getMinPosition() const;

  /**
   * Returns the maximum position the gripper can reach
   */
  float getMaxPosition() const;

  enum eUnitConversion
  {
    TO_DEVICE_UNIT,   //!< TO_DEVICE_UNIT
    FROM_DEVICE_UNIT  //!< FROM_DEVICE_UNIT
  };

  /**
   * Converts the given value
   */
  float convertValueUnit(float Value, eMoveParameter Param, eUnitConversion ConversionDirection) const;

  /**
   * Returns the current device position in the range from 0 - 255
   */
  int getCurrentDevicePosition();

  /**
   * For socket timeouts
   */
  void check_deadline();

 private:
  /**
   * Private move implementation that is called from the public interface functions
   */
  int move_impl(int Position, int Speed, int Force, eMoveMode MoveMode = START_MOVE);
  std::string hostname_;
  int port_;
  bool verbose_;
  ConnectionState conn_state_;
  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
  boost::asio::deadline_timer deadline_;
  int min_position_ = 0;
  int max_position_ = 255;
  int range_mm_ = 40;
  int min_speed_ = 1;  // speed 0 does not make any sense
  int max_speed_ = 255;
  int min_force_ = 0;
  int max_force_ = 255;
  int speed_ = 255;
  int force_ = 0;
  eUnit units_[3] = {UNIT_NORMALIZED, UNIT_NORMALIZED, UNIT_NORMALIZED};
  std::mutex mutex_;
};
}  // namespace ur_rtde

//---------------------------------------------------------------------------
#endif  // ROBOTIQ_GRIPPER_H
