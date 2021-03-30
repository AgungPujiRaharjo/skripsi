cmake_minimum_required(VERSION 3.0.2)
project(program)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ComputeInversKinematics.srv
  ComputeForwardKinematics.srv
  ComputeCOM.srv
  ComputeInversWalk.srv
  GetMagnetoData.srv
  GetMpuData.srv
  GetUltrasonicData.srv

  # Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES program
#  CATKIN_DEPENDS message_generation rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Mark other files for installation (e.g. launch and bag files, etc.)

install(PROGRAMS
  scripts/library/imu_mpu6050.py
  scripts/library/imu_hmc5883l.py
  scripts/library/ultrasonic.py
  scripts/receiver/get_magneto.py
  scripts/receiver/get_mpu.py
  scripts/receiver/get_ultrasonic.py

  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

