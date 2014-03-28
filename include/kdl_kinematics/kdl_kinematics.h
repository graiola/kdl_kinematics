#ifndef KDL_KINEMATICS_H
#define KDL_KINEMATICS_H

#include "ros/ros.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

using namespace std;
using namespace KDL;
using namespace ros;
using namespace Eigen;

class KDLKinematics
{
	public:
		KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max = 0.1, double det_max = 0.0, double epsilon = 0.01);
		
		void ComputeFk(const MatrixXd& joints_pos, Vector3d& position, Matrix3d& orientation);
		void ComputeFk(const VectorXd& joints_pos, Vector3d& position, Matrix3d& orientation);
		void ComputeFk(const VectorXd& joints_pos, VectorXd& pose_pos);
		void ComputeFk(const JntArray& joints_pos, Frame& end_effector);
		
		void ComputeIk(const MatrixXd& joints_pos, const MatrixXd& v_in, MatrixXd& qdot_out);
		void ComputeIk(const VectorXd& joints_pos, const VectorXd& v_in, VectorXd& qdot_out);
		void ComputeIk(const JntArray& joints_pos, const Twist& v_in, JntArray& qdot_out);
		
		void ComputeJac(const MatrixXd& joints_pos, MatrixXd& jac);
		void ComputeJac(const VectorXd& joints_pos, MatrixXd& jac);
		void ComputeJac(const JntArray& joints_pos, Jacobian& jac);
		
		void Twist2MatrixXd(const Twist& in, MatrixXd& out);
		void JntArray2MatrixXd(const JntArray& in, MatrixXd& out);
		void MatrixXd2JntArray(const MatrixXd& in, JntArray& out);
		bool parsed;
	
	protected:
		void PseudoInverse(const Eigen::MatrixXd& a, Eigen::MatrixXd& result);

	private:
		std::string ros_node_name;
		std::string robot_description;
		std::string chain_root_, chain_tip_;
		boost::shared_ptr<ros::NodeHandle> ros_nh;
		Tree kdl_tree;
		Chain kdl_chain;
		JntArray kdl_joints;
		JntArray kdl_joints_dot;
		Jacobian kdl_jacobian_;
		MatrixXd eigen_jacobian_;
		MatrixXd eigen_jacobian_pinv_;
		MatrixXd eigen_cart_dot_;
		MatrixXd eigen_joints_dot_;
		boost::shared_ptr<ChainFkSolverPos_recursive>  kdl_fk_solver_ptr;
		boost::shared_ptr<ChainJntToJacSolver>  kdl_jacobian_solver_ptr;
		int Ndof;
		Frame kdl_end_effector;
		double damp_max_, det_max_, epsilon_, damp_, det_;
		
};

#endif



