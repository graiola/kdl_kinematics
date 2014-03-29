#ifndef KDL_KINEMATICS_H
#define KDL_KINEMATICS_H

////////// ROS
#include "ros/ros.h"

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>

////////// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace kdl_kinematics {

class KDLKinematics
{
	public:
		KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max = 0.1, double det_max = 0.0, double epsilon = 0.01);
		
		void ComputeFk(const Eigen::MatrixXd& joints_pos, Eigen::Vector3d& position, Eigen::Matrix3d& orientation);
		void ComputeFk(const Eigen::VectorXd& joints_pos, Eigen::Vector3d& position, Eigen::Matrix3d& orientation);
		//void ComputeFk(const Eigen::VectorXd& joints_pos, Eigen::VectorXd& pose_pos); Eigen::Ref<Eigen::VectorXd>
		void ComputeFk(const Eigen::Ref<Eigen::VectorXd> joints_pos, Eigen::Ref<Eigen::VectorXd> pose_pos);
		void ComputeFk(const KDL::JntArray& joints_pos, KDL::Frame& end_effector);
		
		void ComputeIk(const Eigen::MatrixXd& joints_pos, const Eigen::MatrixXd& v_in, Eigen::MatrixXd& qdot_out);
		//void ComputeIk(const Eigen::VectorXd& joints_pos, const Eigen::VectorXd& v_in, Eigen::VectorXd& qdot_out);
		void ComputeIk(const Eigen::Ref<Eigen::VectorXd> joints_pos, const Eigen::Ref<Eigen::VectorXd> v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);
		void ComputeIk(const KDL::JntArray& joints_pos, const KDL::Twist& v_in, KDL::JntArray& qdot_out);
		
		void ComputeJac(const Eigen::MatrixXd& joints_pos, Eigen::MatrixXd& jac);
		//void ComputeJac(const Eigen::VectorXd& joints_pos, Eigen::MatrixXd& jac);
		void ComputeJac(const Eigen::Ref<Eigen::VectorXd> joints_pos, Eigen::MatrixXd& jac);
		void ComputeJac(const KDL::JntArray& joints_pos, KDL::Jacobian& jac);
		
		void Twist2MatrixXd(const KDL::Twist& in, Eigen::MatrixXd& out);
		void JntArray2MatrixXd(const KDL::JntArray& in, Eigen::MatrixXd& out);
		void MatrixXd2JntArray(const Eigen::MatrixXd& in, KDL::JntArray& out);
		
		bool isParsed(){return parsed;}
		
		int getNdof(){return Ndof;}
	
	protected:
		void PseudoInverse(const Eigen::MatrixXd& a, Eigen::MatrixXd& result);

	private:
		std::string ros_node_name;
		std::string robot_description;
		std::string chain_root_, chain_tip_;
		boost::shared_ptr<ros::NodeHandle> ros_nh;
		KDL::Tree kdl_tree;
		KDL::Chain kdl_chain;
		KDL::JntArray kdl_joints;
		KDL::JntArray kdl_joints_dot;
		KDL::Jacobian kdl_jacobian_;
		Eigen::MatrixXd eigen_jacobian_;
		Eigen::MatrixXd eigen_jacobian_pinv_;
		Eigen::MatrixXd eigen_cart_dot_;
		Eigen::MatrixXd eigen_joints_dot_;
		boost::shared_ptr<KDL::ChainFkSolverPos_recursive>  kdl_fk_solver_ptr;
		boost::shared_ptr<KDL::ChainJntToJacSolver>  kdl_jacobian_solver_ptr;
		KDL::Frame kdl_end_effector;
		double damp_max_, det_max_, epsilon_, damp_, det_;
		bool parsed;
		int Ndof;
};

}

#endif



