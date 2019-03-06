//#include "ros/ros.h"
#include <intersection_explorer/explorer.hh>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "intersection_explorer");
	intersection_explorer::Explorer exp;
	ros::spin();
	return 0;
}
