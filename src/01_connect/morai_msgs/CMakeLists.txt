cmake_minimum_required(VERSION 2.8.3)
project(morai_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

  add_message_files(
    DIRECTORY msg
    FILES
    CollisionData.msg
    CollisionObject.msg
    CtrlCmd.msg
    EgoVehicleStatus.msg
  )




 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  message_runtime
)

