#ifdef EIGEN_MALLOC_CHECKS
  #define EIGEN_RUNTIME_NO_MALLOC
  #define START_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(false); } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(true); } while (0) 
#else
  #define START_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
#endif

#include <gtest/gtest.h>
#include "kdl_kinematics/kdl_kinematics.h"

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

using namespace kdl_kinematics;
using namespace Eigen;
using namespace boost;

// NOTE: Hardcoded names for meka robot, the robot_description needs to be loaded on the param server!
std::string end_effector_name = "palm_right";
std::string root_name = "T0";

TEST(KDLKinematics, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles

  EXPECT_NO_THROW(KDLKinematics(root_name,end_effector_name));
}

TEST(KDLKinematics, ComputeForwardKinematics)
{
  KDLKinematics kk(root_name,end_effector_name);
  VectorXd joints_pos(kk.getNdof());
  VectorXd qdot(kk.getNdof());
  VectorXd pose(kk.getCartSize());
  VectorXd velocity(kk.getCartSize());
  Vector3d position;
  Matrix3d orientation;
  
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(kk.ComputeFk(joints_pos,pose));
  EXPECT_NO_THROW(kk.ComputeFk(joints_pos,position,orientation));
  EXPECT_NO_THROW(kk.ComputeFkDot(joints_pos,qdot,velocity)); // NOTE: Rt problems with the interface
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(KDLKinematics, ComputeJacobian)
{
  KDLKinematics kk(root_name,end_effector_name);
   
  VectorXd joints_pos(kk.getNdof());
  MatrixXd jacobian(kk.getCartSize(),kk.getNdof());

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(kk.ComputeJac(joints_pos,jacobian));
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(KDLKinematics, ComputeInverseKinematics)
{
  void ComputeIk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);
  
  KDLKinematics kk(root_name,end_effector_name);
   
  VectorXd joints_pos(kk.getNdof());
  VectorXd velocity(kk.getCartSize());
  VectorXd qdot(kk.getNdof());
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(kk.ComputeIk(joints_pos,velocity,qdot));
  END_REAL_TIME_CRITICAL_CODE();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}