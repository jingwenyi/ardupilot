#pragma once

#include "ap_version.h"

#define THISFIRMWARE "Uav-rs copter v2.2.0"
#define FIRMWARE_VERSION 1,2,0,FIRMWARE_VERSION_TYPE_OFFICIAL

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
