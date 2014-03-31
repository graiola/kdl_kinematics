#include "kdl_kinematics/kdl_kinematics.h"

using namespace kdl_kinematics;

// Note: To run this test the meka's robot_description should be correctly loaded on the ros parameter server

typedef double vector_t; 

void testInterfacesEigen(KDLKinematics& kdl_kinematics){
	
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

void testInterfacesStd(KDLKinematics& kdl_kinematics){
	
	// Testing FK interface
	ROS_INFO("TESTING: ComputeFk(joints_pos,pose_pos)");
	std::vector<vector_t> joints_pos = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};
	std::vector<vector_t> pose_pos(6); // x y z r p y
	kdl_kinematics.ComputeFk(joints_pos,pose_pos);
	for(int i = 0; i < pose_pos.size(); i++)
		std::cout << "pose_pos["<<i<<"]: " << pose_pos[i] << std::endl;
	
	ROS_INFO("TESTING: ComputeFk(joints_pos,position,orientation)");
	Eigen::Vector3d position;
	Eigen::Matrix3d orientation;
	kdl_kinematics.ComputeFk(joints_pos,position,orientation);
	ROS_INFO_STREAM("RESULT:\n position:\n" << position << "\n" << "orientation:\n" << orientation);
}

int main(int argc, char *argv[])
{
	// How to create the object:
	std::string end_effector_name = "palm_right";
	std::string root_name = "T0";
	
	try
	{
		KDLKinematics kdl_kinematics(root_name,end_effector_name);
		ROS_INFO("EIGEN INTERFACE");
		testInterfacesEigen(kdl_kinematics);
		ROS_INFO("STD INTERFACE");
		testInterfacesStd(kdl_kinematics);
	}
	catch(const std::runtime_error& e)
	{	
		std::cout << e.what() << std::endl; // FIX, why I can not use ROS_ERROR_STREAM??
		return 0;
	}

	
}