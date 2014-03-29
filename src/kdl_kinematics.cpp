#include "kdl_kinematics/kdl_kinematics.h"

using namespace std;
using namespace KDL;
using namespace ros;
using namespace Eigen;

namespace kdl_kinematics {

void KDLKinematics::Twist2MatrixXd(const Twist& in, MatrixXd& out){
	assert(out.rows() == 6);
	assert(out.cols() == 1);
	
	for(unsigned int i=0; i<3; i++){
		out(i) = in.vel(i);
		out(i+3) = in.rot(i);
	}
	
}

void KDLKinematics::JntArray2MatrixXd(const JntArray& in, MatrixXd& out){
	assert(out.rows() == in.rows()); 
	assert(out.cols() == in.columns()); // columns() Always 1 
	
	for(unsigned int i=0; i<in.rows(); i++){
		out(i) = in(i);
	}
}

void KDLKinematics::MatrixXd2JntArray(const MatrixXd& in, JntArray& out){
	assert(out.rows() == in.rows()); 
	assert(out.columns() == in.cols()); // columns() Always 1 
	
	for(unsigned int i=0; i<in.rows(); i++){
		out(i) = in(i);
	}
}

KDLKinematics::KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max, double det_max, double epsilon):chain_root_(chain_root),chain_tip_(chain_tip),damp_max_(damp_max),det_max_(det_max),epsilon_(epsilon){
	
	assert(damp_max_ >= 0.0);
	assert(det_max_ >= 0.0);
	assert(epsilon_ >= 0.0);
	
	parsed = true;
	ros_node_name = "kdl_kinematics";
	int argc = 1;
	char* arg0 = strdup(ros_node_name.c_str());
	char* argv[] = {arg0, 0};
	init(argc, argv, ros_node_name);
	free (arg0);
	
	ros_nh = boost::make_shared<ros::NodeHandle> ("");
	ros_nh->param("robot_description", robot_description, string());
	
	if (!kdl_parser::treeFromString(robot_description, kdl_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		parsed = false;
	}
	
	if(!kdl_tree.getChain(chain_root_,chain_tip_,kdl_chain)){
		ROS_ERROR("Failed to retrain kdl chain");  
		parsed = false;
	}
	
	kdl_jacobian_solver_ptr = boost::make_shared<ChainJntToJacSolver> (kdl_chain);
	kdl_fk_solver_ptr = boost::make_shared<ChainFkSolverPos_recursive> (kdl_chain);
	
	Ndof = kdl_chain.getNrOfJoints();
	
	// Create joint arrays
	kdl_joints = JntArray(Ndof);
	kdl_joints_dot = JntArray(Ndof);
	// Create the jacobian
	kdl_jacobian_.resize(Ndof);
	
	eigen_cart_dot_.resize(6,1); // FIX, hardcoded 6
	eigen_joints_dot_.resize(Ndof,1);
	
	ros_nh->shutdown();
}

void KDLKinematics::PseudoInverse(const MatrixXd& a, MatrixXd& result){

	if(a.rows()<a.cols()){
		MatrixXd tmp_a = a.transpose();
		MatrixXd tmp_result;
		PseudoInverse(tmp_a,tmp_result);
		result = tmp_result.transpose();		
	}
	
	if(damp_max_ == 0){ // Case 1: No damping
		damp_ = 0.0;
	}
	else if(det_max_ != 0){ // Case 2: Linear damping
		if(a.rows() < a.cols()) // Not tested yet
			det_ = (a.transpose()*a).determinant();
		else
			det_ = (a*a.transpose()).determinant();
		damp_ = damp_max_ - damp_max_ * det_/det_max_;
	}else{ // Case 3: Constant damping
		damp_ = damp_max_;
	}
	
	//Eigen::SVD<Eigen::MatrixXd> svd = a.svd(); // Eigen2
	JacobiSVD<MatrixXd> svd(a, ComputeThinU | ComputeThinV); // Eigen3
	VectorXd vect;
	vect = svd.singularValues();
	for (int i = 0; i < vect.size(); i++){
		if(std::abs(vect(i))>epsilon)
			vect(i) = 1.0/vect(i);
		else
			vect(i) = vect(i)/(vect(i)*vect(i)+damp_*damp_);
	}
	result = svd.matrixV() * vect.asDiagonal() * svd.matrixU().transpose();	
}

void KDLKinematics::ComputeIk(const MatrixXd& joints_pos, const MatrixXd& v_in, MatrixXd& qdot_out){
	ComputeJac(joints_pos,eigen_jacobian_);
	PseudoInverse(eigen_jacobian_,eigen_jacobian_pinv_);
	qdot_out = eigen_jacobian_pinv_ * v_in;
}

//void KDLKinematics::ComputeIk(const VectorXd& joints_pos, const VectorXd& v_in, VectorXd& qdot_out){
void KDLKinematics::ComputeIk(const Eigen::Ref<Eigen::VectorXd> joints_pos, const Eigen::Ref<Eigen::VectorXd> v_in, Eigen::Ref<Eigen::VectorXd> qdot_out){
	ComputeJac(joints_pos,eigen_jacobian_);
	PseudoInverse(eigen_jacobian_,eigen_jacobian_pinv_);
	qdot_out = eigen_jacobian_pinv_ * v_in;
}

void KDLKinematics::ComputeIk(const JntArray& joints_pos, const Twist& v_in, JntArray& qdot_out){
	ComputeJac(joints_pos,kdl_jacobian_);
	eigen_jacobian_ = kdl_jacobian_.data;
	PseudoInverse(eigen_jacobian_,eigen_jacobian_pinv_);
	Twist2MatrixXd(v_in,eigen_cart_dot_);
	eigen_joints_dot_ = eigen_jacobian_pinv_ * eigen_cart_dot_;
	MatrixXd2JntArray(eigen_joints_dot_,qdot_out);
}

void KDLKinematics::ComputeFk(const MatrixXd& joints_pos, Vector3d& position, Matrix3d& orientation){
	for(int i = 0; i<Ndof; i++)
		kdl_joints(i) = joints_pos(i);

	kdl_fk_solver_ptr->JntToCart(kdl_joints,kdl_end_effector);
	
	//position = kdl_end_effector.p.data; //FIX Does it work?
	//orientation = kdl_end_effector.M.data;
	
	for(int i = 0; i<3; i++){
		position(i) = kdl_end_effector.p(i);
		for(int j = 0; j<3; j++)
			orientation(i,j) = kdl_end_effector.M(i,j);
	}
}

void KDLKinematics::ComputeFk(const VectorXd& joints_pos, Vector3d& position, Matrix3d& orientation){
	for(int i = 0; i<Ndof; i++)
		kdl_joints(i) = joints_pos(i);

	kdl_fk_solver_ptr->JntToCart(kdl_joints,kdl_end_effector);
	
	//position = kdl_end_effector.p.data; //FIX Does it work?
	//orientation = kdl_end_effector.M.data;
	
	for(int i = 0; i<3; i++){
		position(i) = kdl_end_effector.p(i);
		for(int j = 0; j<3; j++)
			orientation(i,j) = kdl_end_effector.M(i,j);
	}
}

//void KDLKinematics::ComputeFk(const VectorXd& joints_pos, VectorXd& pose_pos){
void KDLKinematics::ComputeFk(const Eigen::Ref<Eigen::VectorXd> joints_pos, Eigen::Ref<Eigen::VectorXd> pose_pos){
	assert(joints_pos.size() >= Ndof);
	assert(pose_pos.size() >= 6);
	for(int i = 0; i<Ndof; i++)
		kdl_joints(i) = joints_pos(i);

	kdl_fk_solver_ptr->JntToCart(kdl_joints,kdl_end_effector);
	
	for(int i = 0; i<3; i++)
		pose_pos(i) = kdl_end_effector.p(i);
	
	kdl_end_effector.M.GetRPY(pose_pos(3),pose_pos(4),pose_pos(5));
}

void KDLKinematics::ComputeFk(const JntArray& joints_pos, Frame& end_effector){
	kdl_fk_solver_ptr->JntToCart(joints_pos,end_effector);
}

void KDLKinematics::ComputeJac(const MatrixXd& joints_pos, MatrixXd& jac){
	for(int i = 0; i<Ndof; i++)
		kdl_joints(i) = joints_pos(i);
		
	kdl_jacobian_solver_ptr->JntToJac(kdl_joints,kdl_jacobian_);   
	
	jac = kdl_jacobian_.data;
}

//void KDLKinematics::ComputeJac(const VectorXd& joints_pos, MatrixXd& jac){
void KDLKinematics::ComputeJac(const Eigen::Ref<Eigen::VectorXd> joints_pos, MatrixXd& jac){	
	
	for(int i = 0; i<Ndof; i++)
		kdl_joints(i) = joints_pos(i);

	kdl_jacobian_solver_ptr->JntToJac(kdl_joints,kdl_jacobian_);   
	
	jac = kdl_jacobian_.data;
}

void KDLKinematics::ComputeJac(const JntArray& joints_pos, Jacobian& jac){	
	kdl_jacobian_solver_ptr->JntToJac(joints_pos,jac);   
}

}