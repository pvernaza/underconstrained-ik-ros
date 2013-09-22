#ifndef __UnderconstrainedIKROS_hh__
#define __UnderconstrainedIKROS_hh__

#include "UnderconstrainedIK.hh"

#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>

namespace underconstrained_ik_ros {

  typedef boost::shared_ptr<robot_state::JointStateGroup> KinModel;
  typedef std::pair<std::string, Eigen::Vector3d> KinChainPt;

  template<>
  class KinematicModelOps<KinModel, KinChainPt> {

  public:

    static 
    Eigen::Vector3d 
    ComputeCartesianFK(KinModel const& kinModel, 
		       KinChainPt const& kinChainPt,
		       std::vector<UCJointIKState> const& jointIKStates) {
      Eigen::Affine3d const& ptState = 
	kinModel
	->getRobotState()
	->getLinkState(kinChainPt.first)
	->getGlobalLinkTransform();
      Eigen::Vector3d res = ptState.TranslationPart();
    }
    
    static 
    Eigen::MatrixXd 
    ComputeJacobian(KinModel const& kinModel, 
		    KinChainPt const& kinChainPt,
		    std::vector<UCJointIKState> const& jointIKStates) {
      Eigen::MatrixXd jacobian;
      kinModel->getJacobian(kinChainPt.first, kinChainPt.second, jacobian);
      return jacobian;
    }
  };

  class UCIKSolver : public UCIKSolver<KinModel, KinChainPt> {

  };

};

#endif
