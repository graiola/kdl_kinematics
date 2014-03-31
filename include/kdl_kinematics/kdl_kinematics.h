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
	
typedef Eigen::JacobiSVD<Eigen::MatrixXd> svd_t;

class KDLKinematics
{
	public:
		KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max = 0.1, double det_max = 0.0, double epsilon = 0.01);
		
		void ComputeFk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, Eigen::Ref<Eigen::Vector3d> position, Eigen::Ref<Eigen::Matrix3d> orientation);
		void ComputeFk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, Eigen::Ref<Eigen::VectorXd> pose_pos);
		
		void ComputeIk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);

		bool isParsed(){return parsed;}
		int getNdof(){return Ndof_;}
		
		// External use
		Eigen::MatrixXd getInvJacobian(){return eigen_jacobian_;}
		Eigen::MatrixXd getJacobian(){return eigen_jacobian_pinv_;}
	
	protected: // For internal use (they use preallocated variables)
		void PseudoInverse(); 
		void ComputeJac();

	private:
		std::string ros_node_name_;
		std::string robot_description_;
		std::string chain_root_, chain_tip_;
		boost::shared_ptr<ros::NodeHandle> ros_nh_;
		KDL::Tree kdl_tree_;
		KDL::Chain kdl_chain_;
		KDL::JntArray kdl_joints_;
		KDL::Jacobian kdl_jacobian_;
		Eigen::MatrixXd eigen_jacobian_;
		Eigen::MatrixXd eigen_jacobian_pinv_;
		boost::shared_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_ptr_;
		boost::shared_ptr<KDL::ChainJntToJacSolver> kdl_jacobian_solver_ptr_;
		KDL::Frame kdl_end_effector_;
		double damp_max_, det_max_, epsilon_, damp_, det_;
		bool parsed;
		int Ndof_;
		boost::shared_ptr<svd_t> svd_;
		Eigen::VectorXd svd_vect_;
		
};

}

#endif



