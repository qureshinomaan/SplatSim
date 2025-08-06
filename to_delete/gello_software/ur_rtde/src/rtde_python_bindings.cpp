#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_control_interface_doc.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_io_interface_doc.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_receive_interface_doc.h>
#include <ur_rtde/script_client.h>

namespace py = pybind11;
using namespace ur_rtde;

namespace rtde_control
{
PYBIND11_MODULE(rtde_control, m)
{
  m.doc() = "RTDE Control Interface";

  py::class_<PathEntry> pentry(m, "PathEntry");
  pentry.def(py::init<PathEntry::eMoveType, PathEntry::ePositionType, std::vector<double>>(), py::arg("move_type"), py::arg("position_type"), py::arg("parameters"))
      .def("toScriptCode", &PathEntry::toScriptCode, "", py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const PathEntry &a) { return "<rtde_control.PathEntry>"; });
  py::enum_<PathEntry::eMoveType>(pentry, "eMoveType")
      .value("MoveJ", PathEntry::eMoveType::MoveJ)
      .value("MoveL", PathEntry::eMoveType::MoveL)
      .value("MoveP", PathEntry::eMoveType::MoveP)
      .value("MoveC", PathEntry::eMoveType::MoveC)
      .export_values();
  py::enum_<PathEntry::ePositionType>(pentry, "ePositionType")
      .value("PositionTcpPose", PathEntry::ePositionType::PositionTcpPose)
      .value("PositionJoints", PathEntry::ePositionType::PositionJoints)
      .export_values();

  py::class_<Path>(m, "Path")
      .def(py::init<>())
      .def("addEntry", &Path::addEntry, "", py::arg("entry"), py::call_guard<py::gil_scoped_release>())
      .def("clear", &Path::clear, "", py::call_guard<py::gil_scoped_release>())
      .def("size", &Path::size, "", py::call_guard<py::gil_scoped_release>())
      .def("waypoints", &Path::waypoints, "", py::call_guard<py::gil_scoped_release>())
      .def("appendMovelPath", &Path::appendMovelPath, "", py::arg("path"), py::call_guard<py::gil_scoped_release>())
      .def("appendMovejPath", &Path::appendMovejPath, "", py::arg("path"), py::call_guard<py::gil_scoped_release>())
      .def("toScriptCode", &Path::toScriptCode, "", py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const Path &a) { return "<rtde_control.Path>"; });

  py::class_<AsyncOperationStatus>(m, "AsyncOperationStatus")
	.def(py::init<int>())
	.def("value", &AsyncOperationStatus::value, "", py::call_guard<py::gil_scoped_release>())
	.def("isAsyncOperationRunning", &AsyncOperationStatus::isAsyncOperationRunning, "", py::call_guard<py::gil_scoped_release>())
	.def("operationId", &AsyncOperationStatus::operationId, "", py::call_guard<py::gil_scoped_release>())
	.def("changeCount", &AsyncOperationStatus::changeCount, "", py::call_guard<py::gil_scoped_release>())
	.def("progress", &AsyncOperationStatus::progress, "", py::call_guard<py::gil_scoped_release>())
	.def("equals", &AsyncOperationStatus::equals, py::arg("other"), py::call_guard<py::gil_scoped_release>())
	.def("__repr__", [](const AsyncOperationStatus &a) { return "<rtde_control.AsyncOperationStatus>"; });


  py::class_<RTDEControlInterface> control(m, "RTDEControlInterface");
  py::enum_<RTDEControlInterface::Flags>(control, "Flags", py::arithmetic())
      .value("FLAG_UPLOAD_SCRIPT", RTDEControlInterface::Flags::FLAG_UPLOAD_SCRIPT)
      .value("FLAG_USE_EXT_UR_CAP", RTDEControlInterface::Flags::FLAG_USE_EXT_UR_CAP)
      .value("FLAG_VERBOSE", RTDEControlInterface::Flags::FLAG_VERBOSE)
      .value("FLAG_UPPER_RANGE_REGISTERS", RTDEControlInterface::Flags::FLAG_UPPER_RANGE_REGISTERS)
      .value("FLAG_NO_WAIT", RTDEControlInterface::Flags::FLAG_NO_WAIT)
      .value("FLAG_CUSTOM_SCRIPT", RTDEControlInterface::Flags::FLAG_CUSTOM_SCRIPT)
      .value("FLAGS_DEFAULT", RTDEControlInterface::Flags::FLAGS_DEFAULT)
      .export_values();

  py::enum_<RTDEControlInterface::Feature>(control, "Feature")
      .value("FEATURE_BASE", RTDEControlInterface::Feature::FEATURE_BASE)
      .value("FEATURE_TOOL", RTDEControlInterface::Feature::FEATURE_TOOL)
      .value("FEATURE_CUSTOM", RTDEControlInterface::Feature::FEATURE_CUSTOM)
      .export_values();

  control.def(py::init<std::string, double, uint16_t, int, int>(), py::arg("hostname"),
	          py::arg("frequency") = -1.0,
              py::arg("flags") = RTDEControlInterface::Flags::FLAGS_DEFAULT,
              py::arg("ur_cap_port") = 50002, py::arg("rt_priority") = 0);
  control.def("disconnect", &RTDEControlInterface::disconnect, DOC(ur_rtde, RTDEControlInterface, disconnect),py::call_guard<py::gil_scoped_release>());
  control.def("reconnect", &RTDEControlInterface::reconnect, DOC(ur_rtde, RTDEControlInterface, reconnect),
           py::call_guard<py::gil_scoped_release>());
  control.def("isConnected", &RTDEControlInterface::isConnected, DOC(ur_rtde, RTDEControlInterface, isConnected),
           py::call_guard<py::gil_scoped_release>());
  control.def("sendCustomScriptFunction", &RTDEControlInterface::sendCustomScriptFunction,
           DOC(ur_rtde, RTDEControlInterface, sendCustomScriptFunction), py::call_guard<py::gil_scoped_release>());
  control.def("sendCustomScript", &RTDEControlInterface::sendCustomScript,
           DOC(ur_rtde, RTDEControlInterface, sendCustomScript), py::call_guard<py::gil_scoped_release>());
  control.def("sendCustomScriptFile", &RTDEControlInterface::sendCustomScriptFile,
           DOC(ur_rtde, RTDEControlInterface, sendCustomScriptFile), py::call_guard<py::gil_scoped_release>());
  control.def("setCustomScriptFile", &RTDEControlInterface::setCustomScriptFile,
           DOC(ur_rtde, RTDEControlInterface, setCustomScriptFile), py::call_guard<py::gil_scoped_release>());
  control.def("stopScript", &RTDEControlInterface::stopScript, DOC(ur_rtde, RTDEControlInterface, stopScript),
           py::call_guard<py::gil_scoped_release>());
  control.def("reuploadScript", &RTDEControlInterface::reuploadScript, DOC(ur_rtde, RTDEControlInterface, reuploadScript),
           py::call_guard<py::gil_scoped_release>());
  control.def("moveJ",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path, bool asynchronous)) & RTDEControlInterface::moveJ,
           DOC(ur_rtde, RTDEControlInterface, moveJ_2), py::arg("path"), py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("movePath",
           (bool (RTDEControlInterface::*)(const Path &path, bool asynchronous)) & RTDEControlInterface::movePath, DOC(ur_rtde, RTDEControlInterface, movePath),
           "", py::arg("path"), py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("moveJ",
           (bool (RTDEControlInterface::*)(const std::vector<double> &q, double speed, double acceleration, bool asynchronous)) &
               RTDEControlInterface::moveJ,
           DOC(ur_rtde, RTDEControlInterface, moveJ), py::arg("q"), py::arg("speed") = 1.05,
           py::arg("acceleration") = 1.4, py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("moveJ_IK", &RTDEControlInterface::moveJ_IK, DOC(ur_rtde, RTDEControlInterface, moveJ_IK), py::arg("pose"),
           py::arg("speed") = 1.05, py::arg("acceleration") = 1.4, py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path, bool asynchronous)) & RTDEControlInterface::moveL,
           DOC(ur_rtde, RTDEControlInterface, moveL_2), py::arg("path"), py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<double> &pose, double speed, double acceleration, bool asynchronous)) &
               RTDEControlInterface::moveL,
           DOC(ur_rtde, RTDEControlInterface, moveL), py::arg("pose"), py::arg("speed") = 0.25,
           py::arg("acceleration") = 1.2, py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("moveL_FK", &RTDEControlInterface::moveL_FK, DOC(ur_rtde, RTDEControlInterface, moveL_FK), py::arg("q"),
           py::arg("speed") = 0.25, py::arg("acceleration") = 1.2, py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("speedJ", &RTDEControlInterface::speedJ, DOC(ur_rtde, RTDEControlInterface, speedJ), py::arg("qd"),
           py::arg("acceleration") = 0.5, py::arg("time") = 0.0, py::call_guard<py::gil_scoped_release>());
  control.def("speedL", &RTDEControlInterface::speedL, DOC(ur_rtde, RTDEControlInterface, speedL), py::arg("xd"),
           py::arg("acceleration") = 0.25, py::arg("time") = 0.0, py::call_guard<py::gil_scoped_release>());
  control.def("speedStop", &RTDEControlInterface::speedStop, py::arg("a") = 10.0, DOC(ur_rtde, RTDEControlInterface, speedStop),
           py::call_guard<py::gil_scoped_release>());
  control.def("servoJ", &RTDEControlInterface::servoJ, DOC(ur_rtde, RTDEControlInterface, servoJ),
           py::call_guard<py::gil_scoped_release>());
  control.def("servoL", &RTDEControlInterface::servoL, DOC(ur_rtde, RTDEControlInterface, servoL),
           py::call_guard<py::gil_scoped_release>());
  control.def("servoC", &RTDEControlInterface::servoC, DOC(ur_rtde, RTDEControlInterface, servoC), py::arg("pose"),
           py::arg("speed") = 0.25, py::arg("acceleration") = 1.2, py::arg("blend") = 0.0,
           py::call_guard<py::gil_scoped_release>());
  control.def("servoStop", &RTDEControlInterface::servoStop, py::arg("a") = 10.0, DOC(ur_rtde, RTDEControlInterface, servoStop),
           py::call_guard<py::gil_scoped_release>());
  control.def("forceMode", &RTDEControlInterface::forceMode, DOC(ur_rtde, RTDEControlInterface, forceMode),
           py::call_guard<py::gil_scoped_release>());
  control.def("forceModeStop", &RTDEControlInterface::forceModeStop, DOC(ur_rtde, RTDEControlInterface, forceModeStop),
           py::call_guard<py::gil_scoped_release>());
  control.def("forceModeSetDamping", &RTDEControlInterface::forceModeSetDamping,
           DOC(ur_rtde, RTDEControlInterface, forceModeSetDamping), py::call_guard<py::gil_scoped_release>());
  control.def("toolContact", &RTDEControlInterface::toolContact, DOC(ur_rtde, RTDEControlInterface, toolContact),
           py::call_guard<py::gil_scoped_release>());
  control.def("getTargetWaypoint", &RTDEControlInterface::getTargetWaypoint,
           DOC(ur_rtde, RTDEControlInterface, getTargetWaypoint), py::call_guard<py::gil_scoped_release>());
  control.def("getActualJointPositionsHistory", &RTDEControlInterface::getActualJointPositionsHistory,
           DOC(ur_rtde, RTDEControlInterface, getActualJointPositionsHistory), py::call_guard<py::gil_scoped_release>());
  control.def("getStepTime", &RTDEControlInterface::getStepTime, DOC(ur_rtde, RTDEControlInterface, getStepTime),
           py::call_guard<py::gil_scoped_release>());
  control.def("teachMode", &RTDEControlInterface::teachMode, DOC(ur_rtde, RTDEControlInterface, teachMode),
           py::call_guard<py::gil_scoped_release>());
  control.def("endTeachMode", &RTDEControlInterface::endTeachMode, DOC(ur_rtde, RTDEControlInterface, endTeachMode),
           py::call_guard<py::gil_scoped_release>());
  control.def("isProgramRunning", &RTDEControlInterface::isProgramRunning, DOC(ur_rtde, RTDEControlInterface, isProgramRunning),
           py::call_guard<py::gil_scoped_release>());
  control.def("forceModeSetGainScaling", &RTDEControlInterface::forceModeSetGainScaling,
           DOC(ur_rtde, RTDEControlInterface, forceModeSetGainScaling), py::call_guard<py::gil_scoped_release>());
  control.def("zeroFtSensor", &RTDEControlInterface::zeroFtSensor, DOC(ur_rtde, RTDEControlInterface, zeroFtSensor),
           py::call_guard<py::gil_scoped_release>());
  control.def("setPayload", &RTDEControlInterface::setPayload, DOC(ur_rtde, RTDEControlInterface, setPayload),
           py::call_guard<py::gil_scoped_release>());
  control.def("setTcp", &RTDEControlInterface::setTcp, DOC(ur_rtde, RTDEControlInterface, setTcp),
           py::call_guard<py::gil_scoped_release>());
  control.def("getInverseKinematics", &RTDEControlInterface::getInverseKinematics,
           DOC(ur_rtde, RTDEControlInterface, getInverseKinematics),
              py::arg("x"), py::arg("qnear") = std::vector<double>(),
           py::arg("max_position_error") = 1e-10, py::arg("max_orientation_error") = 1e-10,
           py::call_guard<py::gil_scoped_release>());
  control.def("poseTrans", &RTDEControlInterface::poseTrans,
           DOC(ur_rtde, RTDEControlInterface, poseTrans), py::arg("p_from"), py::arg("p_from_to"),
           py::call_guard<py::gil_scoped_release>());
  control.def("triggerProtectiveStop", &RTDEControlInterface::triggerProtectiveStop,
           DOC(ur_rtde, RTDEControlInterface, triggerProtectiveStop), py::call_guard<py::gil_scoped_release>());
  control.def("stopL", &RTDEControlInterface::stopL,
           DOC(ur_rtde, RTDEControlInterface, stopL), py::arg("a") = 10.0, py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("stopJ", &RTDEControlInterface::stopJ,
           DOC(ur_rtde, RTDEControlInterface, stopJ), py::arg("a") = 2.0, py::arg("asynchronous") = false, py::call_guard<py::gil_scoped_release>());
  control.def("setWatchdog", &RTDEControlInterface::setWatchdog,
           DOC(ur_rtde, RTDEControlInterface, setWatchdog), py::arg("min_frequency") = 10.0,
           py::call_guard<py::gil_scoped_release>());
  control.def("kickWatchdog", &RTDEControlInterface::kickWatchdog,
           DOC(ur_rtde, RTDEControlInterface, kickWatchdog), py::call_guard<py::gil_scoped_release>());
  control.def("isPoseWithinSafetyLimits", &RTDEControlInterface::isPoseWithinSafetyLimits,
           DOC(ur_rtde, RTDEControlInterface, isPoseWithinSafetyLimits), py::call_guard<py::gil_scoped_release>());
  control.def("isJointsWithinSafetyLimits", &RTDEControlInterface::isJointsWithinSafetyLimits,
           DOC(ur_rtde, RTDEControlInterface, isJointsWithinSafetyLimits), py::call_guard<py::gil_scoped_release>());
  control.def("getJointTorques", &RTDEControlInterface::getJointTorques,
           DOC(ur_rtde, RTDEControlInterface, getJointTorques), py::call_guard<py::gil_scoped_release>());
  control.def("getTCPOffset", &RTDEControlInterface::getTCPOffset, DOC(ur_rtde, RTDEControlInterface, getTCPOffset),
              py::call_guard<py::gil_scoped_release>());
  control.def("getForwardKinematics", &RTDEControlInterface::getForwardKinematics, DOC(ur_rtde, RTDEControlInterface, getForwardKinematics),
              py::arg("q") = std::vector<double>(),py::arg("tcp_offset") = std::vector<double>(),
                  py::call_guard<py::gil_scoped_release>());
  control.def("isSteady", &RTDEControlInterface::isSteady, DOC(ur_rtde, RTDEControlInterface, isSteady), py::call_guard<py::gil_scoped_release>());
  control.def("moveUntilContact", &RTDEControlInterface::moveUntilContact, DOC(ur_rtde, RTDEControlInterface, moveUntilContact), py::arg("xd"),
              py::arg("direction") = std::vector<double>{0, 0, 0, 0, 0, 0}, py::arg("acceleration") = 1.2,
              py::call_guard<py::gil_scoped_release>());
  control.def("jogStart", &RTDEControlInterface::jogStart, DOC(ur_rtde, RTDEControlInterface, jogStart), py::arg("speeds"),
              py::arg("feature") = RTDEControlInterface::Feature::FEATURE_BASE,
              py::arg("acc") = 0.5,
              py::arg("custom_frame") = std::vector<double>{0, 0, 0, 0, 0, 0},
              py::call_guard<py::gil_scoped_release>());
  control.def("jogStop", &RTDEControlInterface::jogStop, DOC(ur_rtde, RTDEControlInterface, jogStop), py::call_guard<py::gil_scoped_release>());
  control.def("freedriveMode", &RTDEControlInterface::freedriveMode, py::arg("free_axes") = std::vector<int>{1, 1, 1, 1, 1, 1},
              py::arg("feature") = std::vector<double>{0, 0, 0, 0, 0, 0}, py::call_guard<py::gil_scoped_release>());
  control.def("endFreedriveMode", &RTDEControlInterface::endFreedriveMode, py::call_guard<py::gil_scoped_release>());
  control.def("getFreedriveStatus", &RTDEControlInterface::getFreedriveStatus, py::call_guard<py::gil_scoped_release>());
  control.def("ftRtdeInputEnable", &RTDEControlInterface::ftRtdeInputEnable, py::arg("enable"),
              py::arg("sensor_mass") = 0.0,
              py::arg("sensor_measuring_offset") = std::vector<double>{0.0, 0.0, 0.0},
              py::arg("sensor_cog") = std::vector<double>{0.0, 0.0, 0.0},
              py::call_guard<py::gil_scoped_release>());
  control.def("enableExternalFtSensor", &RTDEControlInterface::enableExternalFtSensor, py::arg("enable"),
              py::arg("sensor_mass") = 0.0,
              py::arg("sensor_measuring_offset") = std::vector<double>{0.0, 0.0, 0.0},
              py::arg("sensor_cog") = std::vector<double>{0.0, 0.0, 0.0},
              py::call_guard<py::gil_scoped_release>());
  control.def("setExternalForceTorque", &RTDEControlInterface::setExternalForceTorque, py::call_guard<py::gil_scoped_release>());
  control.def("getAsyncOperationProgress", &RTDEControlInterface::getAsyncOperationProgress, DOC(ur_rtde, RTDEControlInterface, getAsyncOperationProgress), py::call_guard<py::gil_scoped_release>());
  control.def("getAsyncOperationProgressEx", &RTDEControlInterface::getAsyncOperationProgressEx, DOC(ur_rtde, RTDEControlInterface, getAsyncOperationProgressEx), py::call_guard<py::gil_scoped_release>());
  control.def("getRobotStatus", &RTDEControlInterface::getRobotStatus, DOC(ur_rtde, RTDEControlInterface, getRobotStatus), py::call_guard<py::gil_scoped_release>());
  control.def("getActualToolFlangePose", &RTDEControlInterface::getActualToolFlangePose, py::call_guard<py::gil_scoped_release>());
  control.def("setGravity", &RTDEControlInterface::setGravity, py::call_guard<py::gil_scoped_release>());
  control.def("initPeriod", &RTDEControlInterface::initPeriod, py::call_guard<py::gil_scoped_release>());
  control.def("waitPeriod", &RTDEControlInterface::waitPeriod, py::call_guard<py::gil_scoped_release>());
  control.def("getInverseKinematicsHasSolution", &RTDEControlInterface::getInverseKinematicsHasSolution,
              py::arg("x"), py::arg("qnear") = std::vector<double>(),
              py::arg("max_position_error") = 1e-10, py::arg("max_orientation_error") = 1e-10,
              py::call_guard<py::gil_scoped_release>());
  control.def("__repr__", [](const RTDEControlInterface &a) { return "<rtde_control.RTDEControlInterface>"; });
}
};  // namespace rtde_control

namespace rtde_receive
{
PYBIND11_MODULE(rtde_receive, m)
{
  m.doc() = "RTDE Receive Interface";
  py::class_<RTDEReceiveInterface>(m, "RTDEReceiveInterface")
      .def(py::init<std::string, double, std::vector<std::string>, bool, bool, int>(), py::arg("hostname"),
           py::arg("frequency") = -1.0,
           py::arg("variables") = std::vector<std::string>(), py::arg("verbose") = false,
           py::arg("use_upper_range_registers") = false,
           py::arg("rt_priority") = 0)
      .def("disconnect", &RTDEReceiveInterface::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("reconnect", &RTDEReceiveInterface::reconnect, DOC(ur_rtde, RTDEReceiveInterface, reconnect),
           py::call_guard<py::gil_scoped_release>())
      .def("startFileRecording", &RTDEReceiveInterface::startFileRecording, py::arg("filename"),
           py::arg("variables") = std::vector<std::string>(), py::call_guard<py::gil_scoped_release>())
      .def("stopFileRecording", &RTDEReceiveInterface::stopFileRecording, py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &RTDEReceiveInterface::isConnected, DOC(ur_rtde, RTDEReceiveInterface, isConnected),
           py::call_guard<py::gil_scoped_release>())
      .def("getTimestamp", &RTDEReceiveInterface::getTimestamp, DOC(ur_rtde, RTDEReceiveInterface, getTimestamp),
           py::call_guard<py::gil_scoped_release>())
      .def("getTargetQ", &RTDEReceiveInterface::getTargetQ, DOC(ur_rtde, RTDEReceiveInterface, getTargetQ),
           py::call_guard<py::gil_scoped_release>())
      .def("getTargetQd", &RTDEReceiveInterface::getTargetQd, DOC(ur_rtde, RTDEReceiveInterface, getTargetQd),
           py::call_guard<py::gil_scoped_release>())
      .def("getTargetQdd", &RTDEReceiveInterface::getTargetQdd, DOC(ur_rtde, RTDEReceiveInterface, getTargetQdd),
           py::call_guard<py::gil_scoped_release>())
      .def("getTargetCurrent", &RTDEReceiveInterface::getTargetCurrent,
           DOC(ur_rtde, RTDEReceiveInterface, getTargetCurrent), py::call_guard<py::gil_scoped_release>())
      .def("getTargetMoment", &RTDEReceiveInterface::getTargetMoment,
           DOC(ur_rtde, RTDEReceiveInterface, getTargetMoment), py::call_guard<py::gil_scoped_release>())
      .def("getActualQ", &RTDEReceiveInterface::getActualQ, DOC(ur_rtde, RTDEReceiveInterface, getActualQ),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualQd", &RTDEReceiveInterface::getActualQd, DOC(ur_rtde, RTDEReceiveInterface, getActualQd),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualCurrent", &RTDEReceiveInterface::getActualCurrent,
           DOC(ur_rtde, RTDEReceiveInterface, getActualCurrent), py::call_guard<py::gil_scoped_release>())
      .def("getJointControlOutput", &RTDEReceiveInterface::getJointControlOutput,
           DOC(ur_rtde, RTDEReceiveInterface, getJointControlOutput), py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPPose", &RTDEReceiveInterface::getActualTCPPose,
           DOC(ur_rtde, RTDEReceiveInterface, getActualTCPPose), py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPSpeed", &RTDEReceiveInterface::getActualTCPSpeed,
           DOC(ur_rtde, RTDEReceiveInterface, getActualTCPSpeed), py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPForce", &RTDEReceiveInterface::getActualTCPForce,
           DOC(ur_rtde, RTDEReceiveInterface, getActualTCPForce), py::call_guard<py::gil_scoped_release>())
      .def("getTargetTCPPose", &RTDEReceiveInterface::getTargetTCPPose,
           DOC(ur_rtde, RTDEReceiveInterface, getTargetTCPPose), py::call_guard<py::gil_scoped_release>())
      .def("getTargetTCPSpeed", &RTDEReceiveInterface::getTargetTCPSpeed,
           DOC(ur_rtde, RTDEReceiveInterface, getTargetTCPSpeed), py::call_guard<py::gil_scoped_release>())
      .def("getActualDigitalInputBits", &RTDEReceiveInterface::getActualDigitalInputBits,
           DOC(ur_rtde, RTDEReceiveInterface, getActualDigitalInputBits), py::call_guard<py::gil_scoped_release>())
      .def("getDigitalInState", &RTDEReceiveInterface::getDigitalInState, py::call_guard<py::gil_scoped_release>())
      .def("getJointTemperatures", &RTDEReceiveInterface::getJointTemperatures,
           DOC(ur_rtde, RTDEReceiveInterface, getJointTemperatures), py::call_guard<py::gil_scoped_release>())
      .def("getActualExecutionTime", &RTDEReceiveInterface::getActualExecutionTime,
           DOC(ur_rtde, RTDEReceiveInterface, getActualExecutionTime), py::call_guard<py::gil_scoped_release>())
      .def("getRobotMode", &RTDEReceiveInterface::getRobotMode, DOC(ur_rtde, RTDEReceiveInterface, getRobotMode),
           py::call_guard<py::gil_scoped_release>())
      .def("getJointMode", &RTDEReceiveInterface::getJointMode, DOC(ur_rtde, RTDEReceiveInterface, getJointMode),
           py::call_guard<py::gil_scoped_release>())
      .def("getSafetyMode", &RTDEReceiveInterface::getSafetyMode, DOC(ur_rtde, RTDEReceiveInterface, getSafetyMode),
           py::call_guard<py::gil_scoped_release>())
      .def("getSafetyStatusBits", &RTDEReceiveInterface::getSafetyStatusBits,
           DOC(ur_rtde, RTDEReceiveInterface, getSafetyStatusBits),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualToolAccelerometer", &RTDEReceiveInterface::getActualToolAccelerometer,
           DOC(ur_rtde, RTDEReceiveInterface, getActualToolAccelerometer), py::call_guard<py::gil_scoped_release>())
      .def("getSpeedScaling", &RTDEReceiveInterface::getSpeedScaling,
           DOC(ur_rtde, RTDEReceiveInterface, getSpeedScaling), py::call_guard<py::gil_scoped_release>())
      .def("getTargetSpeedFraction", &RTDEReceiveInterface::getTargetSpeedFraction,
           DOC(ur_rtde, RTDEReceiveInterface, getTargetSpeedFraction), py::call_guard<py::gil_scoped_release>())
      .def("getActualMomentum", &RTDEReceiveInterface::getActualMomentum,
           DOC(ur_rtde, RTDEReceiveInterface, getActualMomentum), py::call_guard<py::gil_scoped_release>())
      .def("getActualMainVoltage", &RTDEReceiveInterface::getActualMainVoltage,
           DOC(ur_rtde, RTDEReceiveInterface, getActualMainVoltage), py::call_guard<py::gil_scoped_release>())
      .def("getActualRobotVoltage", &RTDEReceiveInterface::getActualRobotVoltage,
           DOC(ur_rtde, RTDEReceiveInterface, getActualRobotVoltage), py::call_guard<py::gil_scoped_release>())
      .def("getActualRobotCurrent", &RTDEReceiveInterface::getActualRobotCurrent,
           DOC(ur_rtde, RTDEReceiveInterface, getActualRobotCurrent), py::call_guard<py::gil_scoped_release>())
      .def("getActualJointVoltage", &RTDEReceiveInterface::getActualJointVoltage,
           DOC(ur_rtde, RTDEReceiveInterface, getActualJointVoltage), py::call_guard<py::gil_scoped_release>())
      .def("getActualDigitalOutputBits", &RTDEReceiveInterface::getActualDigitalOutputBits,
           DOC(ur_rtde, RTDEReceiveInterface, getActualDigitalOutputBits), py::call_guard<py::gil_scoped_release>())
      .def("getDigitalOutState", &RTDEReceiveInterface::getDigitalOutState, py::call_guard<py::gil_scoped_release>())
      .def("getRuntimeState", &RTDEReceiveInterface::getRuntimeState,
           DOC(ur_rtde, RTDEReceiveInterface, getRuntimeState), py::call_guard<py::gil_scoped_release>())
      .def("getRobotStatus", &RTDEReceiveInterface::getRobotStatus,
           DOC(ur_rtde, RTDEReceiveInterface, getRobotStatus), py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogInput0", &RTDEReceiveInterface::getStandardAnalogInput0,
           DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogInput0), py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogInput1", &RTDEReceiveInterface::getStandardAnalogInput1,
           DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogInput1), py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogOutput0", &RTDEReceiveInterface::getStandardAnalogOutput0,
           DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogOutput0), py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogOutput1", &RTDEReceiveInterface::getStandardAnalogOutput1,
           DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogOutput1), py::call_guard<py::gil_scoped_release>())
      .def("isProtectiveStopped", &RTDEReceiveInterface::isProtectiveStopped,
           DOC(ur_rtde, RTDEReceiveInterface, isProtectiveStopped), py::call_guard<py::gil_scoped_release>())
      .def("isEmergencyStopped", &RTDEReceiveInterface::isEmergencyStopped,
           DOC(ur_rtde, RTDEReceiveInterface, isEmergencyStopped), py::call_guard<py::gil_scoped_release>())
      .def("getOutputIntRegister", &RTDEReceiveInterface::getOutputIntRegister,
           DOC(ur_rtde, RTDEReceiveInterface, getOutputIntRegister),
           py::call_guard<py::gil_scoped_release>())
      .def("getOutputDoubleRegister", &RTDEReceiveInterface::getOutputDoubleRegister,
           DOC(ur_rtde, RTDEReceiveInterface, getOutputDoubleRegister),
           py::call_guard<py::gil_scoped_release>())
      .def("getSpeedScalingCombined", &RTDEReceiveInterface::getSpeedScalingCombined,
           DOC(ur_rtde, RTDEReceiveInterface, getSpeedScalingCombined),
           py::call_guard<py::gil_scoped_release>())
      .def("getFtRawWrench", &RTDEReceiveInterface::getFtRawWrench,
           py::call_guard<py::gil_scoped_release>())
      .def("getPayload", &RTDEReceiveInterface::getPayload,
           py::call_guard<py::gil_scoped_release>())
      .def("getPayloadCog", &RTDEReceiveInterface::getPayloadCog,
           py::call_guard<py::gil_scoped_release>())
      .def("getPayloadInertia", &RTDEReceiveInterface::getPayloadInertia,
           py::call_guard<py::gil_scoped_release>())
      .def("initPeriod", &RTDEReceiveInterface::initPeriod,
           py::call_guard<py::gil_scoped_release>())
      .def("waitPeriod", &RTDEReceiveInterface::waitPeriod,
           py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEReceiveInterface &a) { return "<rtde_receive.RTDEReceiveInterface>"; });
}
};  // namespace rtde_receive

namespace rtde_io
{
PYBIND11_MODULE(rtde_io, m)
{
  m.doc() = "RTDE IO Interface";
  py::class_<RTDEIOInterface>(m, "RTDEIOInterface")
      .def(py::init<std::string, bool, bool>(), py::arg("hostname"), py::arg("verbose") = false,
           py::arg("use_upper_range_registers") = false)
      .def("reconnect", &RTDEIOInterface::reconnect, DOC(ur_rtde, RTDEIOInterface, reconnect),
           py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &RTDEIOInterface::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("setStandardDigitalOut", &RTDEIOInterface::setStandardDigitalOut,
           DOC(ur_rtde, RTDEIOInterface, setStandardDigitalOut), py::call_guard<py::gil_scoped_release>())
      .def("setToolDigitalOut", &RTDEIOInterface::setToolDigitalOut, DOC(ur_rtde, RTDEIOInterface, setToolDigitalOut),
           py::call_guard<py::gil_scoped_release>())
      .def("setSpeedSlider", &RTDEIOInterface::setSpeedSlider, DOC(ur_rtde, RTDEIOInterface, setSpeedSlider),
           py::call_guard<py::gil_scoped_release>())
      .def("setAnalogOutputVoltage", &RTDEIOInterface::setAnalogOutputVoltage,
           DOC(ur_rtde, RTDEIOInterface, setAnalogOutputVoltage), py::call_guard<py::gil_scoped_release>())
      .def("setAnalogOutputCurrent", &RTDEIOInterface::setAnalogOutputCurrent,
           DOC(ur_rtde, RTDEIOInterface, setAnalogOutputCurrent), py::call_guard<py::gil_scoped_release>())
      .def("setConfigurableDigitalOut", &RTDEIOInterface::setConfigurableDigitalOut,
           DOC(ur_rtde, RTDEIOInterface, setConfigurableDigitalOut),
           py::call_guard<py::gil_scoped_release>())
      .def("setInputIntRegister", &RTDEIOInterface::setInputIntRegister,
           DOC(ur_rtde, RTDEIOInterface, setInputIntRegister),
           py::call_guard<py::gil_scoped_release>())
      .def("setInputDoubleRegister", &RTDEIOInterface::setInputDoubleRegister,
           DOC(ur_rtde, RTDEIOInterface, setInputDoubleRegister),
           py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEIOInterface &a) { return "<rtde_io.RTDEIOInterface>"; });
}
};  // namespace rtde_io

namespace script_client
{
PYBIND11_MODULE(script_client, m)
{
  m.doc() = "Script Client";
  py::class_<ScriptClient>(m, "ScriptClient")
      .def(py::init<std::string, uint32_t, uint32_t, int, bool>(), py::arg("hostname"),
           py::arg("major_control_version"), py::arg("minor_control_version"), py::arg("port") = 30002,
           py::arg("verbose") = false)
      .def("connect", &ScriptClient::connect, py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &ScriptClient::isConnected, py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &ScriptClient::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("setScriptFile", (bool (ScriptClient::*)()) & ScriptClient::setScriptFile,
           py::call_guard<py::gil_scoped_release>())
      .def("sendScript", (bool (ScriptClient::*)()) & ScriptClient::sendScript,
           py::call_guard<py::gil_scoped_release>())
      .def("sendScript", (bool (ScriptClient::*)(const std::string &file_name)) & ScriptClient::sendScript,
           py::call_guard<py::gil_scoped_release>())
      .def("sendScriptCommand", &ScriptClient::sendScriptCommand, py::call_guard<py::gil_scoped_release>())
      .def("getScript", &ScriptClient::getScript, py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const ScriptClient &a) { return "<script_client.ScriptClient>"; });
}
};  // namespace script_client

namespace dashboard_client
{
PYBIND11_MODULE(dashboard_client, m)
{
  m.doc() = "Dashboard Client";
  py::class_<DashboardClient>(m, "DashboardClient")
      .def(py::init<std::string, int, bool>(), py::arg("hostname"), py::arg("port") = 29999, py::arg("verbose") = false)
      .def("connect", &DashboardClient::connect, py::arg("timeout_ms") = 2000, py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &DashboardClient::isConnected, py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &DashboardClient::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("send", &DashboardClient::send, py::call_guard<py::gil_scoped_release>())
      .def("receive", &DashboardClient::receive, py::call_guard<py::gil_scoped_release>())
      .def("loadURP", &DashboardClient::loadURP, py::call_guard<py::gil_scoped_release>())
      .def("play", &DashboardClient::play, py::call_guard<py::gil_scoped_release>())
      .def("stop", &DashboardClient::stop, py::call_guard<py::gil_scoped_release>())
      .def("pause", &DashboardClient::pause, py::call_guard<py::gil_scoped_release>())
      .def("quit", &DashboardClient::quit, py::call_guard<py::gil_scoped_release>())
      .def("shutdown", &DashboardClient::shutdown, py::call_guard<py::gil_scoped_release>())
      .def("running", &DashboardClient::running, py::call_guard<py::gil_scoped_release>())
      .def("popup", &DashboardClient::popup, py::call_guard<py::gil_scoped_release>())
      .def("closePopup", &DashboardClient::closePopup, py::call_guard<py::gil_scoped_release>())
      .def("closeSafetyPopup", &DashboardClient::closeSafetyPopup, py::call_guard<py::gil_scoped_release>())
      .def("powerOn", &DashboardClient::powerOn, py::call_guard<py::gil_scoped_release>())
      .def("powerOff", &DashboardClient::powerOff, py::call_guard<py::gil_scoped_release>())
      .def("brakeRelease", &DashboardClient::brakeRelease, py::call_guard<py::gil_scoped_release>())
      .def("unlockProtectiveStop", &DashboardClient::unlockProtectiveStop, py::call_guard<py::gil_scoped_release>())
      .def("restartSafety", &DashboardClient::restartSafety, py::call_guard<py::gil_scoped_release>())
      .def("polyscopeVersion", &DashboardClient::polyscopeVersion, py::call_guard<py::gil_scoped_release>())
      .def("programState", &DashboardClient::programState, py::call_guard<py::gil_scoped_release>())
      .def("robotmode", &DashboardClient::robotmode, py::call_guard<py::gil_scoped_release>())
      .def("getRobotModel", &DashboardClient::getRobotModel, py::call_guard<py::gil_scoped_release>())
      .def("getLoadedProgram", &DashboardClient::getLoadedProgram, py::call_guard<py::gil_scoped_release>())
      .def("safetymode", &DashboardClient::safetymode, py::call_guard<py::gil_scoped_release>())
      .def("safetystatus", &DashboardClient::safetystatus, py::call_guard<py::gil_scoped_release>())
      .def("addToLog", &DashboardClient::addToLog, py::call_guard<py::gil_scoped_release>())
      .def("isProgramSaved", &DashboardClient::isProgramSaved, py::call_guard<py::gil_scoped_release>())
      .def("isInRemoteControl", &DashboardClient::isInRemoteControl, py::call_guard<py::gil_scoped_release>())
      .def("setUserRole", &DashboardClient::setUserRole, py::call_guard<py::gil_scoped_release>())
      .def("getSerialNumber", &DashboardClient::getSerialNumber, py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const DashboardClient &a) { return "<dashboard_client.DashboardClient>"; });
}
};  // namespace dashboard_client
