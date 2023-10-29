#ifndef CRI_KEYWORDS_H
#define CRI_KEYWORDS_H

#include <string>

namespace igus_rebel_hw_controller {
namespace cri_keywords {

// start and end of message
const std::string START = "CRISTART";
const std::string END = "CRIEND";

//generic
const std::string TYPE_STATUS = "STATUS";
const std::string TYPE_OPINFO = "OPINFO";
const std::string TYPE_GSIG = "GSIG";
const std::string TYPE_GRIPPERSTATE = "GRIPPERSTATE";
const std::string TYPE_RUNSTATE = "RUNSTATE";
const std::string TYPE_MESSAGE = "MESSAGE";
const std::string TYPE_CMD = "CMD";
const std::string TYPE_CONFIG = "CONFIG";
const std::string TYPE_INFO = "INFO";

// new additions from ROS2 repository
const std::string TYPE_EXECACK = "EXECACK";
const std::string TYPE_EXECPAUSE = "EXECPAUSE";
const std::string TYPE_EXECEND = "EXECEND";
const std::string TYPE_EXECERROR = "EXECERROR";
const std::string TYPE_CMDACK = "CMDACK";
const std::string TYPE_CMDERROR = "CMDERROR";

// status messages types
const std::string STATUS_MODE = "MODE";
const std::string STATUS_POSJOINTSETPOINT = "POSJOINTSETPOINT";
const std::string STATUS_POSJOINTCURRENT = "POSJOINTCURRENT";
const std::string STATUS_POSCARTROBOT = "POSCARTROBOT";
const std::string STATUS_POSCARTPLATTFORM = "POSCARTPLATTFORM";
const std::string STATUS_OVERRIDE = "OVERRIDE";
const std::string STATUS_DIN = "DIN";
const std::string STATUS_DOUT = "DOUT";
const std::string STATUS_ESTOP = "ESTOP";
const std::string STATUS_SUPPLY = "SUPPLY";
const std::string STATUS_CURRENTALL = "CURRENTALL";
const std::string STATUS_CURRENTJOINTS = "CURRENTJOINTS";
const std::string STATUS_ERROR = "ERROR";
const std::string STATUS_KINSTATE = "KINSTATE";

const std::string COMMAND_CONNECT = "Connect";
const std::string COMMAND_RESET = "Reset";
const std::string COMMAND_ENABLE = "Enable";
const std::string COMMAND_DISABLE = "Disable";
const std::string COMMAND_DISCONNECT = "Disconnect";
const std::string COMMAND_MOTIONTYPEJOINT = "MotionTypeJoint";
const std::string COMMAND_MOTIONTYPECARTBASE = "MotionTypeCartBase";

const std::string CONFIG_GETKINEMATICLIMITS = "GetKinematicLimits";
const std::string CONFIG_GETKINEMATICLIMITS_ANSWER = "KinematicLimits";

}  // namespace cri_keywords
}  // namespace igus_rebel_hw_controller

#endif