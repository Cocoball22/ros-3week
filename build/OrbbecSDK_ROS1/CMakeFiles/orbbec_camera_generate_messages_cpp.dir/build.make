# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cona/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cona/catkin_ws/build

# Utility rule file for orbbec_camera_generate_messages_cpp.

# Include the progress variables for this target.
include OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/progress.make

OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/DeviceInfo.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/Extrinsics.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/Metadata.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/IMUInfo.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetBool.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/SetBool.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraParams.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetInt32.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetString.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/SetInt32.h
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/SetString.h


/home/cona/catkin_ws/devel/include/orbbec_camera/DeviceInfo.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/DeviceInfo.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/DeviceInfo.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/DeviceInfo.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/DeviceInfo.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from orbbec_camera/DeviceInfo.msg"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/DeviceInfo.msg -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/Extrinsics.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/Extrinsics.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/Extrinsics.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/Extrinsics.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/Extrinsics.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from orbbec_camera/Extrinsics.msg"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/Extrinsics.msg -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/Metadata.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/Metadata.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/Metadata.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/Metadata.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/Metadata.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from orbbec_camera/Metadata.msg"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/Metadata.msg -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/IMUInfo.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/IMUInfo.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/IMUInfo.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/IMUInfo.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/IMUInfo.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating C++ code from orbbec_camera/IMUInfo.msg"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/IMUInfo.msg -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/GetBool.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/GetBool.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetBool.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/GetBool.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/GetBool.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating C++ code from orbbec_camera/GetBool.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetBool.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/SetBool.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/SetBool.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/SetBool.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/SetBool.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/SetBool.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating C++ code from orbbec_camera/SetBool.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/SetBool.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetCameraInfo.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /opt/ros/noetic/share/sensor_msgs/msg/RegionOfInterest.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /opt/ros/noetic/share/sensor_msgs/msg/CameraInfo.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating C++ code from orbbec_camera/GetCameraInfo.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetCameraInfo.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraParams.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraParams.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetCameraParams.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraParams.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraParams.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating C++ code from orbbec_camera/GetCameraParams.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetCameraParams.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetDeviceInfo.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg/DeviceInfo.msg
/home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating C++ code from orbbec_camera/GetDeviceInfo.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetDeviceInfo.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/GetInt32.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/GetInt32.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetInt32.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/GetInt32.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/GetInt32.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Generating C++ code from orbbec_camera/GetInt32.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetInt32.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/GetString.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/GetString.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetString.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/GetString.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/GetString.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Generating C++ code from orbbec_camera/GetString.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/GetString.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/SetInt32.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/SetInt32.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/SetInt32.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/SetInt32.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/SetInt32.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Generating C++ code from orbbec_camera/SetInt32.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/SetInt32.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

/home/cona/catkin_ws/devel/include/orbbec_camera/SetString.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/cona/catkin_ws/devel/include/orbbec_camera/SetString.h: /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/SetString.srv
/home/cona/catkin_ws/devel/include/orbbec_camera/SetString.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/cona/catkin_ws/devel/include/orbbec_camera/SetString.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cona/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Generating C++ code from orbbec_camera/SetString.srv"
	cd /home/cona/catkin_ws/src/OrbbecSDK_ROS1 && /home/cona/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/cona/catkin_ws/src/OrbbecSDK_ROS1/srv/SetString.srv -Iorbbec_camera:/home/cona/catkin_ws/src/OrbbecSDK_ROS1/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p orbbec_camera -o /home/cona/catkin_ws/devel/include/orbbec_camera -e /opt/ros/noetic/share/gencpp/cmake/..

orbbec_camera_generate_messages_cpp: OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/DeviceInfo.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/Extrinsics.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/Metadata.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/IMUInfo.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetBool.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/SetBool.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraInfo.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetCameraParams.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetDeviceInfo.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetInt32.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/GetString.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/SetInt32.h
orbbec_camera_generate_messages_cpp: /home/cona/catkin_ws/devel/include/orbbec_camera/SetString.h
orbbec_camera_generate_messages_cpp: OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/build.make

.PHONY : orbbec_camera_generate_messages_cpp

# Rule to build all files generated by this target.
OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/build: orbbec_camera_generate_messages_cpp

.PHONY : OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/build

OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/clean:
	cd /home/cona/catkin_ws/build/OrbbecSDK_ROS1 && $(CMAKE_COMMAND) -P CMakeFiles/orbbec_camera_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/clean

OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/depend:
	cd /home/cona/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cona/catkin_ws/src /home/cona/catkin_ws/src/OrbbecSDK_ROS1 /home/cona/catkin_ws/build /home/cona/catkin_ws/build/OrbbecSDK_ROS1 /home/cona/catkin_ws/build/OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : OrbbecSDK_ROS1/CMakeFiles/orbbec_camera_generate_messages_cpp.dir/depend

