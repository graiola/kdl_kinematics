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
typedef std::vector<int> mask_t;

class KDLKinematics
{
	public:
		KDLKinematics(std::string chain_root, std::string chain_tip, double damp_max = 0.1, double epsilon = 0.01);
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
			assert(pose_pos.size() == cart_size_);
			
			//std::cout<<"Pointer inside for pose_pos: "<<&pose_pos<<std::endl;
			//std::cout<<"Pointer inside for joints_pos: "<<&joints_pos<<std::endl;
			
			ComputeFk(joints_pos);
			KDLFRAME2VECTOR(kdl_end_effector_,pose_pos_tmp_);
			ApplyMaskVector(pose_pos_tmp_,pose_pos);
		}
		void ComputeIk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);
		
		template<typename in_vector_t, typename out_vector_t>
		inline void ApplyMaskVector(const in_vector_t& in, out_vector_t& out)
		{
			assert(in.size() == mask_.size());
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
		
		
		std::string ros_node_name_;
		std::string robot_description_;
		std::string chain_root_, chain_tip_;
		boost::shared_ptr<ros::NodeHandle> ros_nh_ptr_;
		KDL::Chain kdl_chain_;
		KDL::JntArray kdl_joints_;
		KDL::Jacobian kdl_jacobian_;
		Eigen::MatrixXd eigen_jacobian_;
		Eigen::MatrixXd eigen_jacobian_pinv_;
		boost::shared_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_ptr_;
		boost::shared_ptr<KDL::ChainJntToJacSolver> kdl_jacobian_solver_ptr_;
		KDL::Frame kdl_end_effector_;
		double damp_max_, epsilon_, damp_, svd_min_, svd_curr_, mask_cnt_;
		int Ndof_, cart_size_;
		boost::shared_ptr<svd_t> svd_;
		Eigen::VectorXd svd_vect_;
		mask_t mask_;
		std::vector<double> pose_pos_tmp_;
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
		}
		
		void setMask(std::string mask_str)
		{
			KDLKinematics::setMask(mask_str);
			// Create a tmp copy of gains_
			gains_tmp_ = gains_;					
			// Resize gains_
			gains_.resize(cart_size_,cart_size_);
			gains_.fill(0.0);
			/*int idx = 0;
			for(unsigned int i = 0; i < mask_.size(); i++)
				if(mask_[i])
				{
					gains_(idx,idx) = gains_tmp_(i,i); //FIXME
					idx++;
				}*/
			ApplyMaskIdentityMatrix(gains_tmp_,gains_);
			// Resizes
			v_.resize(cart_size_);
			actual_pose_.resize(cart_size_);
			desired_pose_.resize(cart_size_);
			// Clear
			v_.fill(0.0);
			actual_pose_.fill(0.0);
			desired_pose_.fill(0.0);
		}
		
		/*void resizeCartAttributes(int size)
		{
			//KDLKinematics::resizeCartAttributes(size);
			v_.resize(size);
			actual_pose_.resize(size);
			desired_pose_.resize(size);
			// Clear
			v_.fill(0.0);
			actual_pose_.fill(0.0);
			desired_pose_.fill(0.0);
		}*/
		
		const Eigen::MatrixXd getGains(){return gains_;}
		
		void clikStatusStep(const Eigen::Ref<const Eigen::VectorXd>& joints_pos){
				// Compute FK
				KDLKinematics::ComputeFk(joints_pos,actual_pose_);
		}
		
		void clikCommandStep(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& desired_pose, Eigen::Ref<Eigen::VectorXd> q_out){
				
				if(desired_pose.size() == 6){
					KDLKinematics::ApplyMaskVector(desired_pose,desired_pose_);
					/*int idx = 0;
					for(unsigned int i=0;i<mask_.size();i++) //FIXME
						if(mask_[i])
						{
							desired_pose_[idx] = desired_pose[i];
							idx++;
						}*/
				}
				else
					desired_pose_ = desired_pose;
				// Compute IK
				v_ = gains_*(desired_pose_-actual_pose_); // TODO + cart_vel_cmd_
				KDLKinematics::ComputeIk(joints_pos,v_,qdot_);
				// Integrate the joints velocities
				q_out = qdot_ * dt_ + joints_pos;
		}
		
		
		/*void clikStep(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& error, Eigen::Ref<Eigen::VectorXd> q_out){
				// Compute IK
				v_ = gains_*(error); // TODO + cart_vel_cmd_
				KDLKinematics::ComputeIk(joints_pos,v_,qdot_);
				// Integrate the joints velocities
				q_out = qdot_ * dt_ + joints_pos;
		}*/
		
	private:
		Eigen::MatrixXd gains_tmp_;
		Eigen::MatrixXd gains_;
		Eigen::VectorXd v_, qdot_, actual_pose_, desired_pose_;
		double dt_;
		
};

}

#endif



