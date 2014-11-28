#ifndef KDL_KINEMATICS_H
#define KDL_KINEMATICS_H

////////// ROS
#include "ros/ros.h"

#ifdef REALTIME_CHECKS
  #define EIGEN_RUNTIME_NO_MALLOC
  #define ENTERING_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(false); } while (0) 
  #define EXITING_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(true); } while (0) 
#else
  #define ENTERING_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
  #define EXITING_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
#endif

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>


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
typedef std::vector<int> mask_t;

class KDLKinematics
{
	public:
		KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max = 0.1, double epsilon = 0.01);
	
		~KDLKinematics(){if(ros_nh_ptr_!=NULL){delete ros_nh_ptr_;}}
		
		inline void ComputeFk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, Eigen::Ref<Eigen::Vector3d> position, Eigen::Ref<Eigen::Matrix3d> orientation)
		{
			ComputeFk(joints_pos);
			for(int i = 0; i<3; i++){
				position(i) = kdl_end_effector_.p(i);
				for(int j = 0; j<3; j++)
					orientation(i,j) = kdl_end_effector_.M(i,j);
			}
		}
		inline void ComputeFk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, Eigen::Ref<Eigen::VectorXd> pose_pos)
		{
			//assert(pose_pos.size() == cart_size_);
			ComputeFk(joints_pos);
			KDLFRAME2VECTOR(kdl_end_effector_,pose_pos_tmp_);
			if(pose_pos.size() == 6){
				pose_pos = pose_pos_tmp_;
			}
			else
			{
				assert(pose_pos.size() == cart_size_);
				ApplyMaskVector(pose_pos_tmp_,pose_pos);
			}
			//ApplyMaskVector(pose_pos_tmp_,pose_pos);
		}
		void ComputeFkDot(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& qdot_in, Eigen::Ref<Eigen::VectorXd> v_out);
		void ComputeIk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);
		
		inline void ApplyMaskVector(const Eigen::Ref<const Eigen::VectorXd>& in, Eigen::Ref<Eigen::VectorXd> out)
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
		void ApplyMaskIdentityMatrix(const Eigen::Ref<const Eigen::MatrixXd>& in, Eigen::Ref<Eigen::MatrixXd> out);
		void ApplyMaskRowMatrix(const Eigen::Ref<const Eigen::MatrixXd>& in, Eigen::Ref<Eigen::MatrixXd> out);
		void ApplyMaskColMatrix(const Eigen::Ref<const Eigen::MatrixXd>& in, Eigen::Ref<Eigen::MatrixXd> out);
		
		void ComputeJac(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, Eigen::Ref<Eigen::MatrixXd> jacobian);

		Eigen::MatrixXd getInvJacobian(){return eigen_jacobian_pinv_;}
		Eigen::MatrixXd getJacobian(){return eigen_jacobian_;}
		Eigen::MatrixXd getSvdVector(){return svd_vect_;}
		int getNdof(){return Ndof_;}
		int getCartSize(){return cart_size_;}
		double getDamp(){return damp_;}
		int getMaskValue(const int idx){return mask_[idx];}
		KDL::Chain getChain(){return kdl_chain_;}
		void setMask(std::string mask_str);
	protected: // For internal use only (they use preallocated variables)
		void PseudoInverse();
		void ComputeJac();
		inline void ComputeFk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos)
		{	ENTERING_REAL_TIME_CRITICAL_CODE();
			assert(joints_pos.size() >= Ndof_);
			for(int i = 0; i<Ndof_; i++)
				kdl_joints_(i) = joints_pos[i];
			kdl_fk_solver_ptr_->JntToCart(kdl_joints_,kdl_end_effector_);
			EXITING_REAL_TIME_CRITICAL_CODE();
		}
		void setCartSize(int size);
		void resizeCartAttributes(int size);
		
		std::string ros_node_name_;
		std::string robot_description_;
		std::string chain_root_, chain_tip_;
		ros::NodeHandle* ros_nh_ptr_;
		//boost::shared_ptr<ros::NodeHandle> ros_nh_ptr_;
		KDL::Chain kdl_chain_;
		KDL::JntArray kdl_joints_;
		KDL::Jacobian kdl_jacobian_;
		Eigen::MatrixXd eigen_jacobian_;
		Eigen::MatrixXd eigen_jacobian_pinv_;
		Eigen::MatrixXd eigen_jacobian_pinv_tmp_;
		Eigen::MatrixXd matrixU_t_, matrixV_;
		boost::shared_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_ptr_;
		boost::shared_ptr<KDL::ChainJntToJacSolver> kdl_jacobian_solver_ptr_;
		KDL::Frame kdl_end_effector_;
		double damp_max_, epsilon_, damp_, svd_min_, svd_curr_, mask_cnt_;
		int Ndof_, cart_size_;
		boost::shared_ptr<svd_t> svd_;

		Eigen::VectorXd svd_vect_, pose_pos_tmp_, pose_vel_tmp_;

		mask_t mask_;
};

class KDLClik: public KDLKinematics
{
	public:
		KDLClik(std::string root, std::string end_effector, double damp_max, double epsilon, Eigen::MatrixXd gains, double dt):KDLKinematics(root,end_effector,damp_max,epsilon)
		{
			assert(gains.rows() == 6);
			assert(gains.cols() == 6);
			assert(dt > 0.0);
			gains_ = gains;
			dt_ = dt;
			qdot_.resize(Ndof_);
			qdot_.fill(0.0);
			
			// Resizes
			v_.resize(6);
			v_tmp_.resize(cart_size_);
			actual_pose_.resize(6);
			desired_pose_.resize(6);
			// Clear
			v_.fill(0.0);
			v_tmp_.fill(0.0);
			actual_pose_.fill(0.0);
			desired_pose_.fill(0.0);
			
		}
		
		void setMask(std::string mask_str)
		{
			KDLKinematics::setMask(mask_str);
			// Create a tmp copy of gains_
			gains_tmp_ = gains_;					
			// Resize gains_
			gains_.resize(cart_size_,cart_size_);
			gains_.fill(0.0);
			ApplyMaskIdentityMatrix(gains_tmp_,gains_);
			// Resizes
			v_.resize(cart_size_);
			v_tmp_.resize(cart_size_);
			actual_pose_.resize(cart_size_);
			desired_pose_.resize(cart_size_);
			// Clear
			v_.fill(0.0);
			v_tmp_.fill(0.0);
			actual_pose_.fill(0.0);
			desired_pose_.fill(0.0);
		}
		
		const Eigen::MatrixXd getGains(){return gains_;}
		
		inline void clikStatusStep(const Eigen::Ref<const Eigen::VectorXd>& joints_pos){
				// Compute FK
				ENTERING_REAL_TIME_CRITICAL_CODE();
				KDLKinematics::ComputeFk(joints_pos,actual_pose_);
				EXITING_REAL_TIME_CRITICAL_CODE();
		}
		
		inline void clikStatusStep(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, Eigen::Ref<Eigen::VectorXd> actual_pose){
				// Compute FK
				ENTERING_REAL_TIME_CRITICAL_CODE();
				KDLKinematics::ComputeFk(joints_pos,actual_pose);
				if(actual_pose.size() == 6){
					KDLKinematics::ApplyMaskVector(actual_pose,actual_pose_);
				}
				else
				{
					assert(actual_pose.size() == cart_size_);
					actual_pose_ = actual_pose;
				}
				EXITING_REAL_TIME_CRITICAL_CODE();
		}
		
		inline void clikCommandStep(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& desired_pose, Eigen::Ref<Eigen::VectorXd> q_out){
				ENTERING_REAL_TIME_CRITICAL_CODE();
				if(desired_pose.size() == 6){
					KDLKinematics::ApplyMaskVector(desired_pose,desired_pose_);
				}
				else
				{
					assert(desired_pose.size() == cart_size_);
					desired_pose_ = desired_pose;
				}
				// Compute IK
				v_tmp_ = desired_pose_ - actual_pose_; // TODO + cart_vel_cmd_
				v_.noalias() = gains_ * v_tmp_;
				KDLKinematics::ComputeIk(joints_pos,v_,qdot_);
				// Integrate the joints velocities
				q_out = qdot_ * dt_ + joints_pos;
				EXITING_REAL_TIME_CRITICAL_CODE();
		}
		
	private:
		Eigen::MatrixXd gains_tmp_;
		Eigen::MatrixXd gains_;
		Eigen::VectorXd v_, v_tmp_, qdot_, actual_pose_, desired_pose_;
		double dt_;
};

}

#endif



