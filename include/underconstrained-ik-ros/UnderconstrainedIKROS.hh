#ifndef __UnderconstrainedIKROS_hh__
#define __UnderconstrainedIKROS_hh__

#include "UnderconstrainedIK.hh"

#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>

namespace underconstrained_ik_ros {

  struct KinModelAndJointGroup {
    robot_model::RobotModelPtr kinModel;
    std::string jointGroupName; 	   // e.g., "right_arm"
  };

  typedef KinModelAndJointGroup KinModel;
  typedef std::pair<std::string, Eigen::Vector3d> KinChainPt;

  template<>
  class KinematicModelOps<KinModel, KinChainPt> {

  public:

    static 
    Eigen::Vector3d 
    ComputeCartesianFK(KinModel const& kinModel, 
		       KinChainPt const& kinChainPt,
		       std::vector<UCJointIKState> const& jointIKStates) {

      // create a robot kinematic state given the kinematic model
      robot_state::RobotStatePtr kinState
	(new robot_state::RobotState(kinModel.kinModel));
      kinState->setToDefaultValues();

      // FIXME: does this need to be deleted?
      // create a joint state group for the given joint group name
      robot_state::JointStateGroup* jointStateGroup = 
	kinState->getJointStateGroup(kinModel.jointGroupName);

      // set the joint angles for the group
      vector<double> jointAngles(jointIKStates.size());
      for (ii = 0; ii < jointAngles.size(); ii++)
	jointAngles[ii] = jointIKStates[ii].angle;
      jointStateGroup->setVariableValues(jointAngles);

      // call FK
      Eigen::Affine3d const& ptState = 
	jointStateGroup
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

      // FIXME: copy-pasting from above
      // create a robot kinematic state given the kinematic model
      robot_state::RobotStatePtr kinState
	(new robot_state::RobotState(kinModel.kinModel));
      kinState->setToDefaultValues();

      // FIXME: does this need to be deleted?
      // create a joint state group for the given joint group name
      robot_state::JointStateGroup* jointStateGroup = 
	kinState->getJointStateGroup(kinModel.jointGroupName);

      // set the joint angles for the group
      vector<double> jointAngles(jointIKStates.size());
      for (ii = 0; ii < jointAngles.size(); ii++)
	jointAngles[ii] = jointIKStates[ii].angle;
      jointStateGroup->setVariableValues(jointAngles);

      Eigen::MatrixXd jacobian;

      kinModel->getJacobian(kinChainPt.first, kinChainPt.second, jacobian);
      return jacobian;
    }
  };

  class UCIKSolver : public UCIKSolver<KinModel, KinChainPt> {

  };

private:

};

#endif
