#include "kdl_kinematics/kdl_kinematics.h"

using namespace kdl_kinematics;

// Note: To run this test the meka's robot_description should be correctly loaded on the ros parameter server

int main(int argc, char *argv[])
{
	// How to create the object
	std::string end_effector = "palm_right";
	std::string root_name = "T0";
	KDLKinematics kdl_kinematics(root_name,end_effector);
	if(kdl_kinematics.isParsed())
		ROS_INFO("Kdl kinematics correctly created");
	else
		return 0;
	
	// Testing FK interface
	ROS_INFO("TESTING: ComputeFk(joints_pos,pose_pos)");
	Eigen::VectorXd joints_pos = Eigen::VectorXd::Ones(kdl_kinematics.getNdof()) * 0.4;
	Eigen::VectorXd pose_pos(6); // x y z r p y
	kdl_kinematics.ComputeFk(joints_pos,pose_pos);
	ROS_INFO_STREAM("RESULT:\n pose_pos:\n" << pose_pos);
	
	ROS_INFO("TESTING: ComputeFk(joints_pos,position,orientation)");
	Eigen::Vector3d position;
	Eigen::Matrix3d orientation;
	kdl_kinematics.ComputeFk(joints_pos,position,orientation);
	ROS_INFO_STREAM("RESULT:\n position:\n" << position << "\n" << "orientation:\n" << orientation);
	
	// Testing IK interface
	ROS_INFO("TESTING: ComputeIk(joints_pos,v_in,qdot_out);");
	Eigen::VectorXd v_in = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd qdot_out(kdl_kinematics.getNdof());
	kdl_kinematics.ComputeIk(joints_pos,v_in,qdot_out);
	ROS_INFO_STREAM("RESULT:\n qdot_out:\n" << qdot_out);
	
	// Testing gets
	Eigen::MatrixXd jac = kdl_kinematics.getJacobian();
	ROS_INFO_STREAM("jac:\n" << jac);
	Eigen::MatrixXd jac_pinv = kdl_kinematics.getInvJacobian();
	ROS_INFO_STREAM("jac_pinv:\n" << jac_pinv);
}