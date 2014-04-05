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
	
	ros_node_name_ = "kdl_kinematics"; // Fix to move in the interface, so I can generate different nodes with different names for each kinematic chain.
	int argc = 1;
	char* arg0 = strdup(ros_node_name_.c_str());
	char* argv[] = {arg0, 0};
	init(argc, argv, ros_node_name_);
	free (arg0);
	
	ros_nh_ptr_ = boost::make_shared<NodeHandle> ("");
	ros_nh_ptr_->param("robot_description", robot_description_, string());
	
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
	
	mask_.resize(6);
	pose_pos_tmp_.resize(6); // It's always x y z r p y, then it will be filtered to obtain the output in computeFk
	setCartSize(6); // Default x y z r p y
	
	ros_nh_ptr_->shutdown();
}

KDLKinematics::~KDLKinematics()
{
}

void KDLKinematics::ComputeIk(const Ref<const VectorXd>& joints_pos, const Ref<const VectorXd>& v_in, Ref<VectorXd> qdot_out)
{
	assert(joints_pos.size() >= Ndof_);
	assert(v_in.size() == cart_size_);
	assert(qdot_out.size() == Ndof_);
	
	for(int i = 0; i<Ndof_; i++)
		kdl_joints_(i) = joints_pos(i);
	
	ComputeJac();
	
	PseudoInverse();
	
	qdot_out = eigen_jacobian_pinv_ * v_in;
}

void KDLKinematics::setMask(string mask_str)
{
	int n_tokens = 0;
	int cart_size = 0;
	boost::char_separator<char> sep(",");
	boost::tokenizer<boost::char_separator<char>> tokens(mask_str, sep);
	for (const auto& t : tokens)
	{
		if(t == "1" || t == "0")
			n_tokens++;
	}
	
	// First check the mask_str size
	if(n_tokens!=6){
		ROS_INFO("Invalid mask format");
		ROS_INFO("It will be used the default mask 1,1,1,1,1,1");
		for (int i = 0; i<mask_.size(); i++) 
			mask_[i] = 1;
		return;
	}
	else
	{
		n_tokens = 0;
		for (const auto& t : tokens){
			if(t == "1"){
				mask_[n_tokens] = 1;
				cart_size++;
			}
			else
				mask_[n_tokens] = 0;
			std::cout << mask_[n_tokens] << std::endl;
			n_tokens++;
		}	
	}
	
	// Third define the new cartesian dimension i.e fix all the hardcoded six
	setCartSize(cart_size);
	
	/*
	std::cout << "cart_size_" << std::endl;
	std::cout << cart_size_ << std::endl;
	std::cout << "eigen_jacobian_" << std::endl;
	std::cout << eigen_jacobian_ << std::endl;
	std::cout << "eigen_jacobian_pinv_" << std::endl;
	std::cout << eigen_jacobian_pinv_ << std::endl;
	std::cout << "svd_vect_" << std::endl;
	std::cout << svd_vect_ << std::endl;
	*/
}

void KDLKinematics::setCartSize(int size)
{
	cart_size_ = size;
	resizeCartAttributes(size);
}

void KDLKinematics::resizeCartAttributes(int size)
{
	eigen_jacobian_.resize(size,Ndof_);
	eigen_jacobian_pinv_.resize(Ndof_,size);
	svd_.reset(new svd_t(size,Ndof_)); // FIX, prb this is not rt safe
	svd_vect_.resize(size);
}

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
	for (int i = 0; i < svd_vect_.size(); i++)
	{
		if(std::abs(svd_vect_(i))>epsilon_)
			svd_vect_(i) = 1.0/svd_vect_(i);
		else
			svd_vect_(i) = svd_vect_(i)/(svd_vect_(i)*svd_vect_(i)+damp_*damp_);
	}
	
	
	eigen_jacobian_pinv_ = svd_->matrixV() * svd_vect_.asDiagonal() * svd_->matrixU().transpose();	
}

void KDLKinematics::ComputeJac()
{
	kdl_jacobian_solver_ptr_->JntToJac(kdl_joints_,kdl_jacobian_);
	
	if(cart_size_ == 6)
		eigen_jacobian_ = kdl_jacobian_.data;
	else
	{// Apply the mask
		int idx = 0;
		for(int i = 0; i < mask_.size(); i++)
			 if(mask_[i])
			 {
				eigen_jacobian_.row(idx) = kdl_jacobian_.data.row(i);
				idx++;
			 }
			
	}
}

}

