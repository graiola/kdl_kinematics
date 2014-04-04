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

////////// BOOST
#include <boost/tokenizer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

////////// KDLFRAME2VECTOR
#define KDLFRAME2VECTOR(kdl_frame_in,vector_out) do { for(int i = 0; i<3; i++) vector_out[i] = kdl_frame_in.p(i); kdl_frame_in.M.GetRPY(vector_out[3],vector_out[4],vector_out[5]); } while (0)

namespace kdl_kinematics {
	
typedef Eigen::JacobiSVD<Eigen::MatrixXd> svd_t;

class KDLKinematics
{
	public:
		KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max = 0.1, double det_max = 0.0, double epsilon = 0.01);
		
		~KDLKinematics();
		
		template<typename in_vector_t>
		inline void ComputeFk(const in_vector_t& joints_pos, Eigen::Ref<Eigen::Vector3d> position, Eigen::Ref<Eigen::Matrix3d> orientation)
		{
			ComputeFk(joints_pos);
			for(int i = 0; i<3; i++){
				position(i) = kdl_end_effector_.p(i);
				for(int j = 0; j<3; j++)
					orientation(i,j) = kdl_end_effector_.M(i,j);
			}
		}
		template<typename in_vector_t, typename out_vector_t>
		inline void ComputeFk(const in_vector_t& joints_pos, out_vector_t& pose_pos)
		{
			assert(pose_pos.size() >= cart_size_);
			ComputeFk(joints_pos);
			KDLFRAME2VECTOR(kdl_end_effector_,pose_pos);
		}
		template<typename in_vector_t>
		inline void ComputeFk(const in_vector_t& joints_pos, Eigen::Ref<Eigen::VectorXd> pose_pos) // Unfortunally Eigen::Ref can not be templated as reference argument
		{
			assert(pose_pos.size() >= cart_size_);
			ComputeFk(joints_pos);
			KDLFRAME2VECTOR(kdl_end_effector_,pose_pos);
		}

		
		void ComputeIk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);

		int getNdof(){return Ndof_;}
		
		Eigen::MatrixXd getInvJacobian(){return eigen_jacobian_pinv_;}
		Eigen::MatrixXd getJacobian(){return eigen_jacobian_;}
		
		void setMask(std::string mask_str);
	
	protected: // For internal use only (they use preallocated variables)
		void PseudoInverse();
		bool PseudoInverse(const Eigen::MatrixXd& a, Eigen::MatrixXd& result, double damp_max, double det_max, double epsilon);
		void ComputeJac();
		template<typename in_vector_t>
		inline void ComputeFk(const in_vector_t& joints_pos)
		{
			assert(joints_pos.size() >= Ndof_);
			for(int i = 0; i<Ndof_; i++)
				kdl_joints_(i) = joints_pos[i];
			kdl_fk_solver_ptr_->JntToCart(kdl_joints_,kdl_end_effector_);
		}
		void setCartSize(int size);
		void resizeCartAttributes(int size);
	private:
		std::string ros_node_name_;
		std::string robot_description_;
		std::string chain_root_, chain_tip_;
		boost::shared_ptr<ros::NodeHandle> ros_nh_ptr_;
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
		int Ndof_, cart_size_;
		boost::shared_ptr<svd_t> svd_;
		Eigen::VectorXd svd_vect_;
		std::vector<int> mask_;
		
};

}

#endif



