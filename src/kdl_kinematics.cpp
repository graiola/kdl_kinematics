#include "kdl_kinematics/kdl_kinematics.h"

using namespace std;
using namespace KDL;
using namespace ros;
using namespace Eigen;

namespace kdl_kinematics {

KDLKinematics::KDLKinematics(string chain_root, string chain_tip, double damp_max, double det_max, double epsilon):chain_root_(chain_root),chain_tip_(chain_tip),damp_max_(damp_max),det_max_(det_max),epsilon_(epsilon)
{
	assert(damp_max_ >= 0.0);
	assert(det_max_ >= 0.0);
	assert(epsilon_ >= 0.0);
	
	ros_node_name_ = "kdl_kinematics";
	int argc = 1;
	char* arg0 = strdup(ros_node_name_.c_str());
	char* argv[] = {arg0, 0};
	init(argc, argv, ros_node_name_);
	free (arg0);
	
	ros_nh_ = boost::make_shared<NodeHandle> ("");
	ros_nh_->param("robot_description", robot_description_, string());
	
	if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)){
		std::string err("Exception catched during kdl_kinematics initialization: impossible to retrain the robot description and to create the kdl tree");
		//ROS_ERROR_STREAM(err);
		throw std::runtime_error(err);
	}
	
	if(!kdl_tree_.getChain(chain_root_,chain_tip_,kdl_chain_)){
		std::string err("Exception catched during kdl_kinematics initialization: impossible to get the kdl chain from " + chain_root_ +" to " + chain_tip_);
		//ROS_ERROR_STREAM(err);
		throw std::runtime_error(err);
	}
	
	kdl_jacobian_solver_ptr_ = boost::make_shared<ChainJntToJacSolver> (kdl_chain_);
	kdl_fk_solver_ptr_ = boost::make_shared<ChainFkSolverPos_recursive> (kdl_chain_);
	
	Ndof_ = kdl_chain_.getNrOfJoints();
	
	// Create joint arrays
	kdl_joints_ = JntArray(Ndof_);
	// Create the jacobians
	kdl_jacobian_.resize(Ndof_);
	eigen_jacobian_.resize(6,Ndof_);
	eigen_jacobian_pinv_.resize(Ndof_,6);
	
	svd_ = boost::make_shared<svd_t> (6,Ndof_);
	svd_vect_.resize(6);
	
	ros_nh_->shutdown();
}

void KDLKinematics::ComputeIk(const Ref<const VectorXd>& joints_pos, const Ref<const VectorXd>& v_in, Ref<VectorXd> qdot_out)
{
	assert(joints_pos.size() >= Ndof_);
	assert(v_in.size() >= 6);
	assert(qdot_out.size() >= Ndof_);
	
	for(int i = 0; i<Ndof_; i++)
		kdl_joints_(i) = joints_pos(i);
	
	ComputeJac();
	PseudoInverse();
	
	qdot_out = eigen_jacobian_pinv_ * v_in;
}

/*
void KDLKinematics::ComputeFk(const Ref<const VectorXd>& joints_pos, Ref<Vector3d> position, Ref<Matrix3d> orientation)
{
	assert(joints_pos.size() >= Ndof_);
	//for(int i = 0; i<Ndof_; i++)
	//	kdl_joints_(i) = joints_pos(i);
	//kdl_fk_solver_ptr_->JntToCart(kdl_joints_,kdl_end_effector_);
	
	ComputeFk(joints_pos);
	
	for(int i = 0; i<3; i++){
		position(i) = kdl_end_effector_.p(i);
		for(int j = 0; j<3; j++)
			orientation(i,j) = kdl_end_effector_.M(i,j);
	}
}

void KDLKinematics::ComputeFk(const Ref<const VectorXd>& joints_pos, Ref<VectorXd> pose_pos)
{
	
	assert(pose_pos.size() >= 6);
	//assert(joints_pos.size() >= Ndof_);
	//for(int i = 0; i<Ndof_; i++)
	//	kdl_joints_(i) = joints_pos(i);
	//kdl_fk_solver_ptr_->JntToCart(kdl_joints_,kdl_end_effector_);
	
	ComputeFk(joints_pos);
	
	for(int i = 0; i<3; i++)
		pose_pos(i) = kdl_end_effector_.p(i);
	kdl_end_effector_.M.GetRPY(pose_pos(3),pose_pos(4),pose_pos(5));
}*/

void KDLKinematics::PseudoInverse()
{	
	//eigen_jacobian_pinv_ = eigen_jacobian_.transpose(); // Svd only works with rows > cols fixed in Eigen3 :)
	
	if(damp_max_ == 0)  // Case 1: No damping
		damp_ = 0.0;
	else if(det_max_ != 0) // Case 2: Linear damping
	{
		det_ = (eigen_jacobian_*eigen_jacobian_.transpose()).determinant();
		damp_ = damp_max_ - damp_max_ * det_/det_max_;
	}
	else // Case 3: Constant damping
		damp_ = damp_max_;

	svd_->compute(eigen_jacobian_, ComputeThinU | ComputeThinV);
	svd_vect_ = svd_->singularValues();
	for (int i = 0; i < svd_vect_.size(); i++){
		if(std::abs(svd_vect_(i))>epsilon)
			svd_vect_(i) = 1.0/svd_vect_(i);
		else
			svd_vect_(i) = svd_vect_(i)/(svd_vect_(i)*svd_vect_(i)+damp_*damp_);
	}
	
	eigen_jacobian_pinv_ = svd_->matrixV() * svd_vect_.asDiagonal() * svd_->matrixU().transpose();	
}

void KDLKinematics::ComputeJac()
{
	kdl_jacobian_solver_ptr_->JntToJac(kdl_joints_,kdl_jacobian_);   
	eigen_jacobian_ = kdl_jacobian_.data;
}

}

