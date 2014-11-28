#include "kdl_kinematics/kdl_kinematics.h"

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>

using namespace kdl_kinematics;

// Note: To run this test the meka's robot_description should be correctly loaded on the ros parameter server

typedef double vector_t; 

void testInterfacesEigen(KDLKinematics& kdl_kinematics, int cart_size){
	
	// Testing FK interface
	ROS_INFO("TESTING: ComputeFk(joints_pos,pose_pos)");
	Eigen::VectorXd joints_pos = Eigen::VectorXd::Ones(kdl_kinematics.getNdof()) * 0.4;
	Eigen::VectorXd pose_pos(cart_size); // x y z r p y
	
	//std::cout<<"Pointer outside for pose_pos: "<<&pose_pos<<std::endl;
	//std::cout<<"Pointer outside for joints_pos: "<<&joints_pos<<std::endl;
	
	kdl_kinematics.ComputeFk(joints_pos,pose_pos);
	ROS_INFO_STREAM("RESULT:\n pose_pos:\n" << pose_pos);
	
	ROS_INFO("TESTING: ComputeFk(joints_pos,position,orientation)");
	Eigen::Vector3d position;
	Eigen::Matrix3d orientation;
	kdl_kinematics.ComputeFk(joints_pos,position,orientation);
	ROS_INFO_STREAM("RESULT:\n position:\n" << position << "\n" << "orientation:\n" << orientation);

	// Testing IK interface
	ROS_INFO("TESTING: ComputeIk(joints_pos,v_in,qdot_out);");
	Eigen::VectorXd v_in = Eigen::VectorXd::Zero(cart_size);
	Eigen::VectorXd qdot_out(kdl_kinematics.getNdof());
	kdl_kinematics.ComputeIk(joints_pos,v_in,qdot_out);
	ROS_INFO_STREAM("RESULT:\n qdot_out:\n" << qdot_out);
	
	// Testing gets
	Eigen::MatrixXd jac = kdl_kinematics.getJacobian();
	ROS_INFO_STREAM("jac:\n" << jac);
	Eigen::MatrixXd jac_pinv = kdl_kinematics.getInvJacobian();
	ROS_INFO_STREAM("jac_pinv:\n" << jac_pinv);
}

void testClik(KDLClik& kdl_clik, int cart_size)
{
    Eigen::VectorXd joints_pos = Eigen::VectorXd::Ones(kdl_clik.getNdof()) * 0.4;
    Eigen::VectorXd q_out(kdl_clik.getNdof());
    Eigen::VectorXd pose_pos(cart_size); // x y z r p y
    Eigen::VectorXd desired_pose(cart_size); // x y z r p y
    
    std::cout<<kdl_clik.getCartSize()<<std::endl;
    
    
    desired_pose.fill(0.2);
    ROS_INFO("TESTING: clikStatusStep(joints_pos)");
    kdl_clik.clikStatusStep(joints_pos);
    ROS_INFO("TESTING: clikStatusStep(joints_pos,pose_pos)");
    kdl_clik.clikStatusStep(joints_pos, pose_pos);
    ROS_INFO_STREAM("RESULT:\n pose_pos:\n" << pose_pos);
    ROS_INFO("TESTING: clikCommandStep(joints_pos,desired_pose,q_out)");
    kdl_clik.clikCommandStep(joints_pos,desired_pose,q_out);
    ROS_INFO_STREAM("RESULT:\n q_out:\n" << q_out);

}

/*void testInterfacesStd(KDLKinematics& kdl_kinematics, int cart_size){
	
	// Testing FK interface
	ROS_INFO("TESTING: ComputeFk(joints_pos,pose_pos)");
	std::vector<vector_t> joints_pos = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};
	std::vector<vector_t> pose_pos(cart_size); // x y z r p y
	
	//std::cout<<"Pointer outside for pose_pos: "<<&pose_pos<<std::endl;
	//std::cout<<"Pointer outside for joints_pos: "<<&joints_pos<<std::endl;
	
	kdl_kinematics.ComputeFk(joints_pos,pose_pos);
	for(int i = 0; i < pose_pos.size(); i++)
		std::cout << "pose_pos["<<i<<"]: " << pose_pos[i] << std::endl;
	
	// Testing IK interface
	ROS_INFO("TESTING: ComputeIk(joints_pos,v_in,qdot_out);");
	std::vector<vector_t> v_in(cart_size,0.0); // x y z r p y
	std::vector<vector_t> qdot_out(kdl_kinematics.getNdof(),0.0);
	kdl_kinematics.ComputeIk(joints_pos,v_in,qdot_out);
	for(int i = 0; i < qdot_out.size(); i++)
		std::cout << "qdot_out["<<i<<"]: " << qdot_out[i] << std::endl;
	
	ROS_INFO("TESTING: ComputeFk(joints_pos,position,orientation)");
	Eigen::Vector3d position;
	Eigen::Matrix3d orientation;
	kdl_kinematics.ComputeFk(joints_pos,position,orientation);
	ROS_INFO_STREAM("RESULT:\n position:\n" << position << "\n" << "orientation:\n" << orientation);
}*/

void testMask(KDLKinematics& kdl_kinematics){
	
	kdl_kinematics.setMask("1,0,0,0,0,1");
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "kdl_kinematics_test"); //Init ROS
  
	// How to create the object:
	std::string end_effector_name = "palm_right";
	std::string root_name = "T0";
	Eigen::MatrixXd gains(6,6);
	gains.fill(1.0);
	
	try
	{
		KDLKinematics kdl_kinematics(root_name,end_effector_name);
		KDLClik kdl_clik(root_name,end_effector_name,0.1,0.01,gains,0.001);
		ROS_INFO("TEST KDL_KINEMATICS");
		testInterfacesEigen(kdl_kinematics,6);
		//ROS_INFO("STD INTERFACE");
		//testInterfacesStd(kdl_kinematics,6);
		ROS_INFO("TEST KDL_CLIK");
		testClik(kdl_clik,6);
		ROS_INFO("TEST MASK");
		testMask(kdl_kinematics);
		testInterfacesEigen(kdl_kinematics,2);
	}
	catch(const std::runtime_error& e)
	{	
		std::cout << e.what() << std::endl; // FIX, why I can not use ROS_ERROR_STREAM??
		return 0;
	}

	
}