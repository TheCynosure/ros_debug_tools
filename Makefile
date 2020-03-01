all: install

install:
	mkdir -p /usr/include/ROS_DEBUG_TOOLS
	cp src/ROS_DEBUG_TOOLS/* /usr/include/ROS_DEBUG_TOOLS
	cp src/DEBUG.h /usr/include
