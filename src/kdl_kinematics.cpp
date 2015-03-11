#include "kdl_kinematics/kdl_kinematics.h"
#include <boost/concept_check.hpp>

using namespace std;
using namespace KDL;
using namespace ros;
using namespace Eigen;

namespace kdl_kinematics {

KDLKinematics::KDLKinematics(string chain_root, string chain_tip, double damp_max, double epsilon):chain_root_(chain_root),chain_tip_(chain_tip),damp_max_(damp_max),epsilon_(epsilon)
{	
	assert(damp_max_ >= 0.0);
	assert(epsilon_ >= 0.0);
	
	ros_node_name_ = "kdl_kinematics";
	int argc = 1;
	char* arg0 = strdup(ros_node_name_.c_str());
	char* argv[] = {arg0, 0};
	init(argc, argv, ros_node_name_, init_options::NoSigintHandler);
	free (arg0);
	
	if(ros::master::check()){
		//ros_nh_ptr_ = boost::make_shared<NodeHandle> ("");
		ros_nh_ptr_ = new NodeHandle("");
	}
	else{
	    std::string err("Roscore not found");
	    //ROS_ERROR_STREAM(err);
	    throw std::runtime_error(err);
	}

	ros_nh_ptr_->param("robot_description", robot_description_, string());
	
	KDL::Tree* kdl_tree_ptr_tmp;
	kdl_tree_ptr_tmp = new KDL::Tree();
	
	if (!kdl_parser::treeFromString(robot_description_, *kdl_tree_ptr_tmp)){
		std::string err("Exception catched during kdl_kinematics initialization: impossible to retrain the robot description and to create the kdl tree");
		//ROS_ERROR_STREAM(err);
		throw std::runtime_error(err);
	}
	
	if(!kdl_tree_ptr_tmp->getChain(chain_root_,chain_tip_,kdl_chain_)){
		std::string err("Exception catched during kdl_kinematics initialization: impossible to get the kdl chain from " + chain_root_ +" to " + chain_tip_);
		//ROS_ERROR_STREAM(err);
		throw std::runtime_error(err);
	}
	
	delete kdl_tree_ptr_tmp; // It is no needed anymore
	
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

void KDLKinematics::ComputeIk(const VectorXd& joints_pos, const VectorXd& v_in, VectorXd& qdot_out)
{	
	assert(joints_pos.size() >= Ndof_);
	//assert(v_in.size() == cart_size_);
	assert(qdot_out.size() == Ndof_);
	for(int i = 0; i<Ndof_; i++)
		kdl_joints_(i) = joints_pos(i);
	ComputeJac();
	PseudoInverse();
	if(v_in.size() == 6){
		ApplyMaskVector(v_in,pose_vel_tmp_);
	}
	else
	{	
		assert(v_in.size() == cart_size_);
		pose_vel_tmp_ = v_in;
	}
	
	qdot_out.noalias() = eigen_jacobian_pinv_ * pose_vel_tmp_;
}

void KDLKinematics::ComputeFk(const VectorXd& joints_pos, Vector3d& position, Matrix3d& orientation)
{	
	ComputeFk(joints_pos);
	for(int i = 0; i<3; i++)
	{
		position(i) = kdl_end_effector_.p(i);
		for(int j = 0; j<3; j++)
			orientation(i,j) = kdl_end_effector_.M(i,j);
	}
}
void KDLKinematics::ComputeFk(const VectorXd& joints_pos, VectorXd& pose_pos)
{	
	//assert(pose_pos.size() == cart_size_);
	ComputeFk(joints_pos);
	KDLFRAME2VECTOR(kdl_end_effector_,pose_pos_tmp_);
	if(pose_pos.size() == 6)
	{
		pose_pos = pose_pos_tmp_;
	}
	else
	{
		assert(pose_pos.size() == cart_size_);
		ApplyMaskVector(pose_pos_tmp_,pose_pos);
	}
	//ApplyMaskVector(pose_pos_tmp_,pose_pos);
}

void KDLKinematics::ComputeFkDot(const VectorXd& joints_pos, const VectorXd& qdot_in, VectorXd& v_out)
{	
  
	assert(joints_pos.size() >= Ndof_);
	assert(qdot_in.size() >= Ndof_);
	assert(v_out.size() == cart_size_);

        for(int i = 0; i<Ndof_; i++)
            kdl_joints_(i) = joints_pos[i];

        ComputeJac();

	//if(v_out.size() == cart_size_){
	v_out.noalias() = eigen_jacobian_ * qdot_in;
        //}
        //else
        //{

        //}
        //ApplyMaskVector(pose_pos_tmp_,pose_pos);
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
		for (unsigned int i = 0; i<mask_.size(); i++) 
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
			//std::cout << mask_[n_tokens] << std::endl;
			n_tokens++;
		}	
	}
	
	// Define the new cartesian dimension
	setCartSize(cart_size);
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
	eigen_jacobian_pinv_tmp_.resize(Ndof_,size);
	
	svd_.reset(new svd_t(size,Ndof_)); // FIXME prb this is not rt safe
	svd_->compute(eigen_jacobian_, ComputeThinU | ComputeThinV); // This is not rt safe! We trigger it here to pre-allocate the internal variables
	matrixU_t_.resize(size,size);
	matrixV_.resize(Ndof_,size);
	
	svd_vect_.resize(size);
	pose_vel_tmp_.resize(size);
	// Clear
	eigen_jacobian_.fill(0.0);
	eigen_jacobian_pinv_.fill(0.0);
	eigen_jacobian_pinv_tmp_.fill(0.0);
	svd_vect_.fill(0.0);
	pose_vel_tmp_.fill(0.0);
}

void KDLKinematics::PseudoInverse()
{	
	svd_->compute(eigen_jacobian_, ComputeThinU | ComputeThinV);
	svd_vect_ = svd_->singularValues();
	svd_min_ = svd_vect_.minCoeff();
	
	for (int i = 0; i < svd_vect_.size(); i++)
	{
		svd_curr_ = svd_vect_[i];
		
		damp_ = std::exp(-4/epsilon_*svd_vect_[i])*damp_max_;
		svd_vect_[i] = svd_curr_/(svd_curr_*svd_curr_+damp_*damp_);
	}
	
	matrixU_t_ = svd_->matrixU().transpose();
	matrixV_ = svd_->matrixV();

	//eigen_jacobian_pinv_ = svd_->matrixV() * svd_vect_.asDiagonal() * svd_->matrixU().transpose(); // NOTE this is not rt safe
	
	eigen_jacobian_pinv_tmp_ = svd_->matrixV() * svd_vect_.asDiagonal();
	eigen_jacobian_pinv_.noalias() = eigen_jacobian_pinv_tmp_ * matrixU_t_; // NOTE .noalias() does the trick
}

void KDLKinematics::ComputeJac()
{	
	kdl_jacobian_solver_ptr_->JntToJac(kdl_joints_,kdl_jacobian_);
	
	if(cart_size_ == 6)
		eigen_jacobian_ = kdl_jacobian_.data;
	else
		ApplyMaskRowMatrix(kdl_jacobian_.data,eigen_jacobian_);
}

void KDLKinematics::ComputeJac(const VectorXd& joints_pos, MatrixXd& jacobian)
{	
        assert(joints_pos.size() >= Ndof_);

        for(int i = 0; i<Ndof_; i++)
            kdl_joints_(i) = joints_pos[i];

        ComputeJac();

        jacobian = eigen_jacobian_;
}

void KDLKinematics::ApplyMaskVector(const VectorXd& in, VectorXd& out)
{
	assert(in.size() == static_cast<int>(mask_.size()));
	assert(out.size() == cart_size_);
	mask_cnt_ = 0;
	for(unsigned int i = 0; i < mask_.size(); i++)
		if(getMaskValue(i))
		{
			out[mask_cnt_] = in[i];
			mask_cnt_++;
		}
}

void KDLKinematics::ApplyMaskIdentityMatrix(const MatrixXd& in, MatrixXd& out)
{
	assert(in.rows() == in.cols()); // it is a square matrix
	assert(in.rows() == static_cast<int>(mask_.size()));
	assert(out.rows() == out.cols()); // it is a square matrix
	assert(out.rows() == cart_size_);
	mask_cnt_ = 0;
	for(unsigned int i = 0; i < mask_.size(); i++)
		if(getMaskValue(i))
		{
			out(mask_cnt_,mask_cnt_) = in(i,i);
			mask_cnt_++;
		}
}

void KDLKinematics::ApplyMaskRowMatrix(const MatrixXd& in, MatrixXd& out)
{
	assert(in.rows() == static_cast<int>(mask_.size()));
	assert(out.rows() == cart_size_);
	mask_cnt_ = 0;
	for(unsigned int i = 0; i < mask_.size(); i++)
		if(getMaskValue(i))
		{
			for(int j = 0; j < Ndof_; j++)
			  out(mask_cnt_,j) = in(i,j); 
			//out.row(mask_cnt_) = in.row(i); // It causes dynamic memory allocation
			mask_cnt_++;
		}
}

void KDLKinematics::ApplyMaskColMatrix(const MatrixXd& in, MatrixXd& out)
{
	assert(in.cols() == static_cast<int>(mask_.size()));
	assert(out.cols() == cart_size_);
	mask_cnt_ = 0;
	for(unsigned int i = 0; i < mask_.size(); i++)
		if(getMaskValue(i))
		{
		  	for(int j = 0; j < Ndof_; j++)
			  out(j,mask_cnt_) = in(j,i); 
			//out.col(mask_cnt_) = in.col(i); // It causes dynamic memory allocation
			mask_cnt_++;
		}
}



}


