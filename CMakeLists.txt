cmake_minimum_required(VERSION 2.8.3)
project(client)

# 添加依赖 build
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	sensor_msgs
	cv_bridge
	image_transport
	message_generation
)

find_package(	OpenCV 2 REQUIRED )

# Generate messages in the 'msg' folder
add_message_files(
	FILES
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  	DEPENDENCIES		
	std_msgs
)

# catkin specific configuration 
catkin_package(
  	INCLUDE_DIRS include
  	LIBRARIES client
  	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  	DEPENDS system_lib
)

include_directories( 
	${PROJECT_SOURCE_DIR}/include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS} 
)

add_executable( client_node src/client.cpp )
target_link_libraries( client_node ${catkin_LIBRARIES} ${OpenCV_LIBS} )

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
add_dependencies( client_node client_generate_messages_cpp )







