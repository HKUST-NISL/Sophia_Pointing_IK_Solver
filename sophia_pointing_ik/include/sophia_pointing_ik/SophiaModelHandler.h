#ifndef YIFAN_SOPHIA_MODEL_HANDLER_H
#define YIFAN_SOPHIA_MODEL_HANDLER_H
#define _USE_MATH_DEFINES


#include <ros/ros.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "hr_msgs/TargetPosture.h"
#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <Eigen/Core>
#include <csv.hpp>
#include <random>
#include <limits>

#include <VerboseLevel.hh>


class SophiaModelHandler {

    //Moveit AND OTHERsetup
    private:
        ros::NodeHandle& nh;

        //Robote model and state
        const robot_model_loader::RobotModelLoader robotModelLoader;
        robot_model::RobotModelPtr currentRobotModel_;
        //This is the globally maintained current robot state in the class
        robot_state::RobotStatePtr currentRobotState_;

        //Planner plugin
        boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> plannerPluginLoader;
        planning_interface::PlannerManagerPtr plannerInstance;
        std::string plannerPluginName;

        // For visualization
        rviz_visual_tools::RvizVisualToolsPtr rvizVisualTools_;
        moveit_visual_tools::MoveItVisualTools* moveitVisualTools_;
        inline static std::string planningSceneTopicName = "planning_scene";
        inline static std::string rvizMarkerTopicName = "visualization_markers";
        inline static std::string moveitPlannedTrajTopicName = "move_group/display_planned_path";
        ros::Publisher planningScenePublisher;
        ros::Publisher displayPlannedTrajPublisher;
        inline static int queue_size = 1000;
        inline static unsigned short int VERBOSE_LEVEL = 1;


    
    public:
        SophiaModelHandler(ros::NodeHandle& nh_in, unsigned short int VERBOSE_LEVEL_IN);
        void setupRobotModel();
        void setupPlannerPlugin();
        void setupVisualization();

        void saveEigenTransformToFile(std::string file_path,const std::vector<Eigen::Isometry3d> mat_vec) const;
        void loadEigenTransformFromFile(std::string file_path,std::vector<Eigen::Isometry3d>& mat_vec);

        void saveArmGroupJointsToFile(std::string file_path,const std::vector<std::vector<double>> joints_vec) const;
        void loadArmGroupJointsFromFile(std::string file_path,std::vector<std::vector<double>>& joints_vec);


    //Sophia model specific
    private:
        enum ARM
        {
            LEFT,
            RIGHT
        };

        //Whole body group
        const robot_state::JointModelGroup* fullBodyGroupWithArti_;
        const robot_state::JointModelGroup* fullBodyGroupNoArti_;
        const robot_state::JointModelGroup* fullBodyGroupNoArtiNoRight_;
        const robot_state::JointModelGroup* fullBodyGroupNoArtiNoLeft_;


        //Arm and forearm group consists of artificial joints that works with dedicated ik solvers
        const robot_state::JointModelGroup* rightArmGroup_;
        const robot_state::JointModelGroup* rightForeArmGroup_;

        const robot_state::JointModelGroup* leftArmGroup_;
        const robot_state::JointModelGroup* leftForeArmGroup_;

        //Hand group has no artificial joints
        const robot_state::JointModelGroup* rightHandGroup_;
        const robot_state::JointModelGroup* leftHandGroup_;

        moveit::planning_interface::MoveGroupInterface fullBodyWithArtiMoveGroupInterface;
        moveit::planning_interface::MoveGroupInterface rightHandMoveGroupInterface;
        moveit::planning_interface::MoveGroupInterface leftHandMoveGroupInterface;

        inline static std::string robotModelName = "robot_description";
        inline static std::string fullBodyGroupWithArtiName = "full_body_group";
        inline static std::string fullBodyGroupNoArtiName = "full_body_group_no_artificial";
        inline static std::string fullBodyGroupNoArtiRightName = "full_body_no_artificial_no_right";
        inline static std::string fullBodyGroupNoArtiLeftName = "full_body_no_arti_no_left";
        inline static std::string rightArmEELinkName = "right_wrist_roll";
        inline static std::string leftArmEELinkName = "left_wrist_roll";
        inline static std::string rightArmGroupName = "right_arm_group1";
        inline static std::string leftArmGroupName = "left_arm_group1";
        inline static std::string rightForeArmGroupName = "right_arm_group2";
        inline static std::string leftForeArmGroupName = "left_arm_group2";
        inline static std::string rightHandGroupName = "right_hand_group";
        inline static std::string leftHandGroupName = "left_hand_group";

        inline static std::string fullBodyWithArtiRestConfigName = "home_pose";
        inline static std::string rightHandPointingConfigName = "right_hand_pointing";
        inline static std::string leftHandPointingConfigName = "left_hand_pointing";
        inline static std::string rightHandRestConfigName = "right_hand_rest";
        inline static std::string leftHandRestConfigName = "left_hand_rest";

        //For discretizing the artificial wrist work space
        inline static bool computeWristEnvelopeFlag = false;
        inline static std::string kinematicsDataFolderRelativeToNodeExec = "../../../src/SophiaKinematicsData/";
        inline static std::string rightArmWristEnvelopeFile = "rightArmWristEnvelope.csv";
        inline static std::string rightArmWristEnvelopeJointValsFile = "rightArmWristEnvelopeJointVals.csv";
        inline static std::string leftArmWristEnvelopeFile = "leftArmWristEnvelope.csv";
        inline static std::string leftArmWristEnvelopeJointValsFile = "leftArmWristEnvelopeJointVals.csv";
        std::vector<Eigen::Isometry3d> rightArmWristEnvelope;
        std::vector<std::vector<double>> rightArmWristEnvelopeJointVals;
        std::vector<Eigen::Isometry3d> leftArmWristEnvelope;
        std::vector<std::vector<double>> leftArmWristEnvelopeJointVals;  
        inline static double discretization_step = 5*M_PI/180;

        //For computing the pointing configuration
        inline static double handSizeRough = 0.2;//Unit: meter
        inline static double wristRandomAngularDist = 2*M_PI/180;
        inline static double pointingIKTimeout = 0.3;//Unit: s
        std::default_random_engine rng_generator;
        inline static std::string rightArmWristArtificialJointName = "right_wrist_roll_artificial_joint";
        inline static std::string leftArmWristArtificialJointName = "left_wrist_roll_artificial_joint";
        inline static std::string rightArmWristTrueJointName = "right_wrist_roll_joint";
        inline static std::string leftArmWristTrueJointName = "left_wrist_roll_joint";
        moveit::core::JointModel * rightArmWristTrueJointModel_;
        moveit::core::JointModel * leftArmWristTrueJointModel_;
        inline static double leftWristIndexFingerOffset = 190*M_PI/180;
        inline static double leftWristArtificialOffset = -75*M_PI/180;
        inline static double rightWristIndexFingerOffset = -193*M_PI/180;
        inline static double rightWristArtificialOffset = -75*M_PI/180;
        
        //For planning and parameterizing the path to the pointing configuration
        inline static double suppressDistPlanningThreshold = 0.01;
        inline static double maxSpeedScaleFactor = 1.0;
        inline static double maxAccelarationScaleFactor = 1.0;

        //For executing the planned path by publishing it to hrsdk
        inline static double publish_interval_sec = 0.02;
        inline static std::string hrArmStateTopicName = "/hr/animation/set_arm_state";
        inline static std::string hrArmIgnoreLimitTopicName = "/hr/animation/virtual/ignore_limits";
        ros::Publisher hrArmStatePublisher;
        //The joint names in the following two listed are ordered to have 1-1 correspondence
        //The upper list refers to joint names that blender from hrsdk recognizes,
        //the bottom list refers to joint names that urdf and moveit recognizes.
        inline static std::vector<std::string> armStateJointNamesInBlender = {
            //Right arm
            "R_Shoulder_Pitch",
            "R_Shoulder_Roll",
            "R_Shoulder_Yaw",
            "R_Elbow",
            "R_Wrist_Yaw",
            "R_Wrist_Roll",
            "R_Wrist_Pitch",
            //Right hand
            "R_Index_Finger",
            "R_Middle_Finger",
            "R_Ring_Finger",
            "R_Pinky_Finger",
            "R_Thumb_Finger",
            "R_Thumb_Roll",
            "R_Spreading",
            //Left arm
            "L_Shoulder_Pitch",
            "L_Shoulder_Roll",
            "L_Shoulder_Yaw",
            "L_Elbow",
            "L_Wrist_Yaw",
            "L_Wrist_Roll",
            "L_Wrist_Pitch",
            //Left hand
            "L_Index_Finger",
            "L_Middle_Finger",
            "L_Ring_Finger",
            "L_Pinky_Finger",
            "L_Thumb_Finger",
            "L_Thumb_Roll",
            "L_Spreading"
        };
        inline static std::vector<std::string> armStateJointNamesInURDF = {
            //Right arm
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint",
            "right_elbow_yaw_joint",
            "right_wrist_pitch_joint",
            "right_wrist_roll_joint",//The axis of this joint seems wrong
            //Right hand
            "right_index_proximal_joint",
            "right_middle_proximal_joint",
            "right_ring_proximal_joint",
            "right_pinky_proximal_joint",
            "right_thumb_proximal_joint",
            "right_thumb_base_joint",
            "right_index_base_joint",
            //Left arm
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint",
            "left_elbow_yaw_joint",
            "left_wrist_pitch_joint",
            "left_wrist_roll_joint",//The axis of this joint seems wrong
            //Left hand
            "left_index_proximal_joint",
            "left_middle_proximal_joint",
            "left_ring_proximal_joint",
            "left_pinky_proximal_joint",
            "left_thumb_proximal_joint",
            "left_thumb_base_joint",
            "left_index_base_joint"
        };
        inline static std::vector<double> armStateDirectionalTweak = {
            //Right arm
            1,
            1,
            -1,
            -1,
            1,
            1,
            -1,
            //Right hand - fingers might have scaling issues
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            //Left arm
            1,
            1,
            1,
            -1,
            1,
            1,
            1,
            //Left hand - fingers might have scaling issues
            1,
            1,
            1,
            1,
            1,
            1,
            1,         
        };
        inline static std::vector<double> armStateOffsetTweak = {
            //Right arm
            0,
            0,
            0,
            0,
            M_PI/2,
            0,
            0,
            //Right hand - fingers might have scaling issues
            -35*M_PI/180,
            -20*M_PI/180,
            -15*M_PI/180,
            -5*M_PI/180,
            -15*M_PI/180,
            0,
            0,
            //Left arm
            0,
            0,
            15*M_PI/180,
            0,
            -M_PI/2,
            -10*M_PI/180,
            0,
            //Left hand - fingers might have scaling issues
            -20*M_PI/180,
            -15*M_PI/180,
            -15*M_PI/180,
            -15*M_PI/180,
            -15*M_PI/180,
            0,
            0,         
        };


        //TF Broadcaster and some miscellaneous for debugging
        tf2_ros::StaticTransformBroadcaster tf2_broadcaster;
        inline static const std::string world_frame_name = "tf_world";
        inline static const std::vector<std::string> test_frame_names = 
            std::vector<std::string>({"A_Patient","Pose0","Pose1"});

    private:
        void setupExecution();

        //Miscellaneous
        void computeRoughWristEnvelope(ARM left_or_right = ARM::RIGHT);
        void loadWristEnvelope(ARM left_or_right = ARM::RIGHT);
        void placeSphereRvizMarker(const Eigen::Vector3d &pos,rviz_visual_tools::colors color=rviz_visual_tools::BLUE,rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);
        void placeLineRvizMarker(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,rviz_visual_tools::colors color=rviz_visual_tools::BLUE,rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);
        void visualizeRobotStateInRviz(const moveit::core::RobotState& state_in);
        bool robotStateCollisionCheck(const moveit::core::RobotState& state_in);
        void removeAllVisualMarkers();
        bool visualizePlannedTrajectory(const robot_state::JointModelGroup* planning_group_,const robot_trajectory::RobotTrajectoryPtr& robot_traj_);
        void setFullbodyToRest(moveit::core::RobotState& robot_state)const;
        void setBothHandsToRest(moveit::core::RobotState& robot_state)const;

        //============================================ For Computing a pointing configuration ==================================================================================================================
        //Arm placement when pointing - arm places wrist
        bool computeArmPointingConfiguration(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, ARM left_or_right = ARM::RIGHT);
        int findRefWristPlacementForPointing(const Eigen::Vector3d& poi, ARM left_or_right = ARM::RIGHT);
        double getWristPoiDistRandomThreshold();

        //Forearm placement when pointing - forearm orients the wrist and the hand
        bool computeForeArmPointingConfiguration(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, ARM left_or_right = ARM::RIGHT);
        bool handleWristRollJointOffset(moveit::core::RobotState& pointing_state, ARM left_or_right = ARM::RIGHT);
        double wrapAngle(double revo_in) const;
        
        //Hand configuration when pointing - make the hand looks like pointing
        void setHandPointingConfiguration(moveit::core::RobotState& pointing_state, SophiaModelHandler::ARM left_or_right = ARM::RIGHT);

        //Wrapping up the computation of pointing configuration
        bool computePointingConfiguration(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, ARM left_or_right = ARM::RIGHT, bool check_collision = true);
        bool computePointingConfigurationBothArm(const Eigen::Vector3d& poi,moveit::core::RobotState& pointing_state, bool check_collision = true);
        //====================================================================================================================================================================================



        //============================================ For Motion Planning ==================================================================================================================
        //Plan a trajectory to reach the a given configuration
        bool choosePlanningGroupSuppressExcessiveArmMotion(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& goal_state,
            const robot_state::JointModelGroup*& planning_group_
        )const;
        
        bool planPathToGivenConfiguration(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& goal_state, 
            const robot_state::JointModelGroup* planning_group_, 
            robot_trajectory::RobotTrajectoryPtr& robot_traj_);
        
        bool parametrizePlannedPath(double dur,const robot_state::JointModelGroup* planning_group_, robot_trajectory::RobotTrajectoryPtr& robot_traj_) const;
        //====================================================================================================================================================================================



        //============================================ For Plan Execution ==================================================================================================================
        //Execute the planned trajectory to do pointing
        void publishPlannedPathWayPointToHRSDK(double time_instant,	robot_state::RobotStatePtr& state_to_update_,  const robot_trajectory::RobotTrajectoryPtr& robot_traj_) const;

        void publishRobotStateToHRSDK(const robot_state::RobotState& state_to_pub, const moveit::core::JointModelGroup* group_)const;

        void publishJointValuesToHRSDK(const std::vector<double>& joint_values,const std::vector<std::string>& joint_names)const;

        bool executePlannedTrajectory(const robot_state::JointModelGroup* planning_group_, const robot_trajectory::RobotTrajectoryPtr& robot_traj_);
        //====================================================================================================================================================================================




    //=================================== Interface to allow external control of sophia for cuing actions ===============================================
    private:
        bool keep_forcing = false;
        std::thread* forceSophiaRestThread_ = nullptr;
    
    private:
        void rectifySophiaPoseThread()const;

    public:
        void executeTrajectoryWrapper(robot_trajectory::RobotTrajectoryPtr& robot_traj_msg);

        bool generatePointingConfiguration(const geometry_msgs::TransformStamped& poi,moveit::core::RobotStatePtr& robot_state_);

        bool generateAddressingConfiguration(const geometry_msgs::TransformStamped& poi,moveit::core::RobotStatePtr& robot_state_);

        bool planTrajectoryToTargetConfiguration(
            moveit::core::RobotStateConstPtr& start_state_,
            moveit::core::RobotStateConstPtr& goal_state_, 
            const double duration,
	        robot_trajectory::RobotTrajectoryPtr& robot_traj_);

        moveit::core::RobotStatePtr getRestRobotStateCopy();

        moveit::core::RobotStatePtr getCurrentRobotStateCopy()const;

        std::string getBaseJointName()const;

        void rectifySophiaPoseToRest();

        void stopRectifyingSophiaPoseToRest();




    public:
        //Test pointing configuration
        void testPointing();
        //Test joint mapping between urdf and hrsdk
        void testMappingDir();
        void testMappingOffset();

};






#endif