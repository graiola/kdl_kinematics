#include "kdl_kinematics/kdl_kinematics.h"

using namespace kdl_kinematics;

int main(int argc, char *argv[])
{
	// Test for the ensta meka robot
	KDLKinematics kdl_kinematics("T0","palm_right");
	if(kdl_kinematics.isParsed())
		ROS_INFO("Object correctly created");
	
	KDLKinematics* kdl_kinematics_pntr;
	kdl_kinematics_pntr = new KDLKinematics("T0","palm_right");
	if(kdl_kinematics_pntr->isParsed())
		ROS_INFO("Pntr object correctly created");
	delete kdl_kinematics_pntr;
}