#include <SophiaModelHandler.h>

SophiaModelHandler::SophiaModelHandler(ros::NodeHandle& nh_in, unsigned short int VERBOSE_LEVEL_IN):
nh(nh_in),
robotModelLoader(robotModelName),
fullBodyWithArtiMoveGroupInterface(fullBodyGroupWithArtiName),
rightHandMoveGroupInterface(rightHandGroupName),
leftHandMoveGroupInterface(leftHandGroupName)
{
    //Miscellaneous
    VERBOSE_LEVEL = VERBOSE_LEVEL_IN;

    

    //Moveit setup
    setupRobotModel();
    setupPlannerPlugin();
    setupVisualization();
    setupExecution();

    //Compute wrist envelope
    if(computeWristEnvelopeFlag)
    {
        VERBOSE_1("Press any key to start computing wrist envelope.");
        std::cin.get();
        //Compute a discretized version of the possible transforms of the finger
        computeRoughWristEnvelope(SophiaModelHandler::ARM::RIGHT);
        computeRoughWristEnvelope(SophiaModelHandler::ARM::LEFT);
    }

    //Load the envelop and the corresponding joint configs
    loadWristEnvelope(SophiaModelHandler::ARM::RIGHT);
    loadWristEnvelope(SophiaModelHandler::ARM::LEFT);

    //Test pointing
    testPointing();

    // // //Test mapping
    // // testMappingDir();
    // testMappingOffset();
}

void SophiaModelHandler::setupRobotModel()
{
    currentRobotModel_ = robotModelLoader.getModel();
    currentRobotState_ = robot_state::RobotStatePtr(new robot_state::RobotState(currentRobotModel_));


    fullBodyGroupWithArti_ = currentRobotModel_->getJointModelGroup(fullBodyGroupWithArtiName);
    fullBodyGroupNoArti_ = currentRobotModel_->getJointModelGroup(fullBodyGroupNoArtiName);
    fullBodyGroupNoArtiNoRight_ = currentRobotModel_->getJointModelGroup(fullBodyGroupNoArtiRightName);
    fullBodyGroupNoArtiNoLeft_ = currentRobotModel_->getJointModelGroup(fullBodyGroupNoArtiLeftName);


    rightArmGroup_ = currentRobotModel_->getJointModelGroup(rightArmGroupName);
    rightForeArmGroup_ = currentRobotModel_->getJointModelGroup(rightForeArmGroupName);
    rightArmWristTrueJointModel_ = currentRobotModel_->getJointModel(rightArmWristTrueJointName);
    rightHandGroup_ = currentRobotModel_->getJointModelGroup(rightHandGroupName);

    leftArmGroup_ = currentRobotModel_->getJointModelGroup(leftArmGroupName);
    leftForeArmGroup_ = currentRobotModel_->getJointModelGroup(leftForeArmGroupName);
    leftArmWristTrueJointModel_ = currentRobotModel_->getJointModel(leftArmWristTrueJointName);
    leftHandGroup_ = currentRobotModel_->getJointModelGroup(leftHandGroupName);
}

void SophiaModelHandler::setupPlannerPlugin()
{

    // We will now get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!nh.getParam("planning_plugin", plannerPluginName))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        plannerPluginLoader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        plannerInstance.reset(plannerPluginLoader->createUnmanagedInstance(plannerPluginName));
        if (!plannerInstance->initialize(currentRobotModel_, nh.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << plannerInstance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = plannerPluginLoader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << plannerPluginName << "': " << ex.what() << std::endl
                                                            << "Available plugins: " << ss.str());
    }

}

void SophiaModelHandler::setupVisualization()
{
    planningScenePublisher = nh.advertise<moveit_msgs::PlanningScene>(planningSceneTopicName, queue_size);
    displayPlannedTrajPublisher = nh.advertise<moveit_msgs::DisplayTrajectory>(moveitPlannedTrajTopicName, queue_size);

    //Rviz visual tools
    rvizVisualTools_.reset(new rviz_visual_tools::RvizVisualTools(currentRobotModel_->getModelFrame(),rvizMarkerTopicName));
    //Moveit visual tools
    moveitVisualTools_ = new moveit_visual_tools::MoveItVisualTools(robotModelLoader.getRobotDescription());
}

void SophiaModelHandler::setupExecution()
{
    hrArmStatePublisher = nh.advertise<hr_msgs::TargetPosture>(hrArmStateTopicName, queue_size);
}

void SophiaModelHandler::computeRoughWristEnvelope(SophiaModelHandler::ARM left_or_right)
{
    const robot_state::JointModelGroup* arm_group_; 
    std::string arm_ee_link_name;
    std::string envelope_file_name;
    std::string joint_file_name;
    std::vector<Eigen::Isometry3d> arm_wrist_envelope_vec = std::vector<Eigen::Isometry3d>();
    std::vector<std::vector<double>> arm_wrist_joint_vals_envelope = std::vector<std::vector<double>>();
    
    if(left_or_right ==SophiaModelHandler::ARM::LEFT)
    {//Left arm
        arm_group_ = leftArmGroup_;

        arm_ee_link_name = leftArmEELinkName;

        envelope_file_name = kinematicsDataFolderRelativeToNodeExec + leftArmWristEnvelopeFile;
        joint_file_name = kinematicsDataFolderRelativeToNodeExec + leftArmWristEnvelopeJointValsFile;
    }
    else
    {//Right arm
        arm_group_ = rightArmGroup_;

        arm_ee_link_name = rightArmEELinkName;

        envelope_file_name = kinematicsDataFolderRelativeToNodeExec + rightArmWristEnvelopeFile;
        joint_file_name = kinematicsDataFolderRelativeToNodeExec + rightArmWristEnvelopeJointValsFile;
    }

    //Get joint bounds - all the joints have only one dof.
    std::vector<const moveit::core::JointModel *> activeJoints = arm_group_->getActiveJointModels();

    //Store the FK results
    VERBOSE_1("The envelope file will be saved to %s, the corresponding joint file will be saved to %s.",
        envelope_file_name.c_str(),joint_file_name.c_str());


    //Do discretization and FK
    std::vector<moveit_msgs::JointLimits> bound_0 = activeJoints[0]->getVariableBoundsMsg();    
    std::vector<moveit_msgs::JointLimits> bound_1 = activeJoints[1]->getVariableBoundsMsg();
    std::vector<moveit_msgs::JointLimits> bound_2 = activeJoints[2]->getVariableBoundsMsg();
    std::vector<moveit_msgs::JointLimits> bound_3 = activeJoints[3]->getVariableBoundsMsg();

    removeAllVisualMarkers();
    double iterator[4] = {
        bound_0[0].min_position,
        bound_1[0].min_position,
        bound_2[0].min_position,
        bound_3[0].min_position
        };
    while(iterator[0] < bound_0[0].max_position)
    {
        VERBOSE_1("Current J0 %f, min J0 %f, max J0 %f",
            iterator[0],bound_0[0].min_position,bound_0[0].max_position);

        iterator[1] = bound_1[0].min_position;
        while(iterator[1] < bound_1[0].max_position)
        {
            iterator[2] = bound_2[0].min_position;
            while(iterator[2] < bound_2[0].max_position)
            {
                iterator[3] = bound_3[0].min_position;
                while(iterator[3] < bound_3[0].max_position)
                {
                    //No mimic joint in this group
                    currentRobotState_->setJointGroupPositions(
                        arm_group_->getName(),
                        iterator
                    );


                    if(!robotStateCollisionCheck(*currentRobotState_))
                    {
                    
                        //Save current ee transform to the vector that 
                        //would be writtent to a file later
                        arm_wrist_envelope_vec.push_back(
                            currentRobotState_->getGlobalLinkTransform(arm_ee_link_name));
                    
                        //Save the corresponding joint values
                        std::vector<double> joint_vec_now = 
                            {iterator[0],iterator[1],iterator[2],iterator[3]};
                        arm_wrist_joint_vals_envelope.push_back(joint_vec_now);
                    }



                    // ros::Duration(0.01).sleep();

                    iterator[3] += discretization_step;
                }

                iterator[2] += discretization_step;
            }

            iterator[1] += discretization_step;
        }

        iterator[0] += discretization_step;
    }

    //Save to file
    saveEigenTransformToFile(envelope_file_name,arm_wrist_envelope_vec);
    saveArmGroupJointsToFile(joint_file_name,arm_wrist_joint_vals_envelope);


    //Save in the program as well
    if(left_or_right ==SophiaModelHandler::ARM::LEFT)
    {
        leftArmWristEnvelope = arm_wrist_envelope_vec;
        leftArmWristEnvelopeJointVals = arm_wrist_joint_vals_envelope;
    }
    else
    {
        rightArmWristEnvelope = arm_wrist_envelope_vec;
        rightArmWristEnvelopeJointVals = arm_wrist_joint_vals_envelope;        
    }

    VERBOSE_1("Envelope computation finished.");

}

void SophiaModelHandler::loadWristEnvelope(SophiaModelHandler::ARM left_or_right)
{
    if(left_or_right ==SophiaModelHandler::ARM::LEFT)
    {
        std::string envelope_file_name = kinematicsDataFolderRelativeToNodeExec + leftArmWristEnvelopeFile;
        std::string joint_file_name = kinematicsDataFolderRelativeToNodeExec + leftArmWristEnvelopeJointValsFile;
        loadEigenTransformFromFile(envelope_file_name,leftArmWristEnvelope);
        loadArmGroupJointsFromFile(joint_file_name,leftArmWristEnvelopeJointVals);   
    }
    else
    {
        std::string envelope_file_name = kinematicsDataFolderRelativeToNodeExec + rightArmWristEnvelopeFile;
        std::string joint_file_name = kinematicsDataFolderRelativeToNodeExec + rightArmWristEnvelopeJointValsFile;
        loadEigenTransformFromFile(envelope_file_name,rightArmWristEnvelope);
        loadArmGroupJointsFromFile(joint_file_name,rightArmWristEnvelopeJointVals);        
    }
}

void SophiaModelHandler::saveEigenTransformToFile(std::string file_path,const std::vector<Eigen::Isometry3d> mat_vec) const
{
    const static Eigen::IOFormat CSVFormat(
        Eigen::FullPrecision, 0, 
        ", ", ",");

    std::ofstream file(file_path.c_str());


    for(Eigen::Isometry3d mat : mat_vec )
    {
        Eigen::Matrix4d mat4d = mat.matrix();
        file<<mat4d.format(CSVFormat)<<"\n";
    }

    file.close();
}

void SophiaModelHandler::loadEigenTransformFromFile(std::string file_path,std::vector<Eigen::Isometry3d>& mat_vec)
{
    //Cleanup the vec
    mat_vec = std::vector<Eigen::Isometry3d>();

    csv::CSVFormat format;
    format.no_header();
    csv::CSVReader reader(file_path,format);
    
    for (csv::CSVRow& row: reader) 
    { // Input iterator
        Eigen::Matrix4d mat_from_row;
        unsigned int cnt = 0;
        for (csv::CSVField& field : row) 
        {
            if(cnt >= 16)
            {
                ROS_ERROR("Wrong csv file. Should be matrix 4d.");
                return;
            }

            mat_from_row(cnt/4, cnt%4) = std::stod(field.get());
            cnt ++;
        }

        mat_vec.push_back(Eigen::Isometry3d(mat_from_row));

        // ros::Duration(1.0).sleep();
    }
}

void SophiaModelHandler::saveArmGroupJointsToFile(std::string file_path,const std::vector<std::vector<double>> joints_vec) const
{

    std::ofstream file(file_path.c_str());


    for(std::vector<double> joints : joints_vec )
    {
        for(int i = 0 ; i <  joints.size(); i++)
        {
            file<<joints[i]<<", ";
        }
        file<<joints[joints.size() - 1]<<"\n";
    }

    file.close();
}

void SophiaModelHandler::loadArmGroupJointsFromFile(std::string file_path,std::vector<std::vector<double>>& joints_vec)
{
    //Cleanup the vec
    joints_vec = std::vector<std::vector<double>>();

    csv::CSVFormat format;
    format.no_header();
    csv::CSVReader reader(file_path,format);
    
    for (csv::CSVRow& row: reader) 
    { // Input iterator
        std::vector<double> joints = std::vector<double>();
        for (csv::CSVField& field : row) 
        {
            joints.push_back(std::stod(field.get()));
        }

        joints_vec.push_back(joints);

        // ros::Duration(1.0).sleep();
    }
}

void SophiaModelHandler::visualizeRobotStateInRviz(const moveit::core::RobotState& state_in)
{
    moveit_msgs::PlanningScene scene_msg;
    scene_msg.robot_model_name = robotModelLoader.getRobotDescription();
    scene_msg.is_diff = true;

    moveit_msgs::RobotState robot_state_msg;
    robotStateToRobotStateMsg(state_in, robot_state_msg);
    scene_msg.robot_state = robot_state_msg;

    planningScenePublisher.publish(scene_msg);
    // ros::Duration(0.01).sleep();
}

void SophiaModelHandler::placeSphereRvizMarker(
    const Eigen::Vector3d &pos, 
    rviz_visual_tools::colors color, 
    rviz_visual_tools::scales scale)
{
    rvizVisualTools_->publishSphere(pos,color,scale);
    rvizVisualTools_->trigger();
}

void SophiaModelHandler::placeLineRvizMarker(
    const Eigen::Vector3d &point1, 
    const Eigen::Vector3d &point2,
    rviz_visual_tools::colors color,
    rviz_visual_tools::scales scale)
{
    rvizVisualTools_->publishLine(point1,point2,color,scale);
    rvizVisualTools_->trigger();
}

bool SophiaModelHandler::robotStateCollisionCheck(const moveit::core::RobotState& state_in)
{//True if in collision

    //Construct the scene from the robot model object 
    planning_scene::PlanningScene scene_to_use(currentRobotModel_);  
      
    //For collision check
    scene_to_use.setCurrentState(state_in);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    scene_to_use.checkSelfCollision(collision_request, collision_result);

    return collision_result.collision;
}

void SophiaModelHandler::removeAllVisualMarkers()
{
    moveitVisualTools_->deleteAllMarkers();
    rvizVisualTools_->deleteAllMarkers();
}

int SophiaModelHandler::findRefWristPlacementForPointing(const Eigen::Vector3d& poi, SophiaModelHandler::ARM left_or_right)
{

    const std::vector<Eigen::Isometry3d> * wrist_envelope_vec_ = nullptr;

    if(left_or_right ==SophiaModelHandler::ARM::LEFT)
    {//True: left wrist
        wrist_envelope_vec_ = &leftArmWristEnvelope;
    }
    else
    {//False: right wrist
        wrist_envelope_vec_ = &rightArmWristEnvelope;
    }



    bool found_above_threshold = false;
    int min_dist_index = -1;//Assume the joint vals vec and envelope vec matches

    //Loop over the wrist envelop to find a reasonable reference wrist position
    //that is above the thresh yet closest to the poi
    //We try to find at least one such position.
    while(!found_above_threshold)
    {
        double wrist_poi_dist_thresh = getWristPoiDistRandomThreshold();
        
        Eigen::Vector3d current_wrist_position;
        double current_dist = -1;
        double min_dist = std::numeric_limits<double>::max();
        for(int i = 0 ; i < wrist_envelope_vec_->size(); i++)
        {
            Eigen::Isometry3d current_wrist_transform = (*wrist_envelope_vec_)[i];

            //Get translation vector
            current_wrist_position = current_wrist_transform.matrix().block<3,1>(0,3);
            
            //Compute distance to poi
            current_dist = (current_wrist_position - poi).norm();
            if(current_dist < wrist_poi_dist_thresh)
            {
                continue;
            }
            else
            {
                if(current_dist < min_dist)
                {
                    found_above_threshold = true;
                    min_dist = current_dist;
                    min_dist_index = i;
                }
                else
                {
                    continue;
                }
            }

        }        
    }    

    return min_dist_index;
}

double SophiaModelHandler::getWristPoiDistRandomThreshold()
{
    //Randomly sample an offset value for wrist-poi distance
    std::normal_distribution<double> distribution_wrist_poi_dist(
        0,//Mean is larger than hand size
        0.05*handSizeRough//STD is smaller than hand size
        );
    double maximal_dist_offset = 0.2*handSizeRough;
    double dist_offset = abs(distribution_wrist_poi_dist(rng_generator));
    
    //Truncate this offset
    dist_offset = std::min(maximal_dist_offset,dist_offset);

    //Final dist threshold
    double wrist_poi_dist_thresh = handSizeRough + dist_offset;

    return wrist_poi_dist_thresh;
}

bool SophiaModelHandler::computeArmPointingConfiguration(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, SophiaModelHandler::ARM left_or_right)
{
    //Get the reference wrist placement
    int ref_wrist_placement_index = findRefWristPlacementForPointing(poi,left_or_right);
    if(ref_wrist_placement_index > 0)
    {//Found a ref placement

       
        const robot_state::JointModelGroup* arm_group_ = nullptr; 
        const std::vector<std::vector<double>> * wrist_envelope_vec_ = nullptr;
        if(left_or_right == SophiaModelHandler::ARM::LEFT)
        {//Point by left wrist
            arm_group_ = leftArmGroup_;
            wrist_envelope_vec_ = &leftArmWristEnvelopeJointVals;
        }
        else
        {//Point by right wrist
            arm_group_ = rightArmGroup_;
            wrist_envelope_vec_ = &rightArmWristEnvelopeJointVals;
        }
    

        //Update robot state
        //The states from the envelope computation is assumed to be valid and free of collision 
        //NO mimic joint in this group
        pointing_state.setJointGroupPositions(
            arm_group_,
            (*wrist_envelope_vec_)[ref_wrist_placement_index]);

        //Randomize around this state a bit - this of course
        //could lead to self-collision but we postpone that check till we finish the hand part.
        pointing_state.setToRandomPositionsNearBy(
            arm_group_,
            pointing_state,
            wristRandomAngularDist);
        return true;
    }
    else
    {
        return false;
    }
}

bool SophiaModelHandler::computeForeArmPointingConfiguration(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, SophiaModelHandler::ARM left_or_right)
{
    //Orienting wrist and hand for pointing is relatively simple, just solve the lookat3d ik
    //But again we do a collision check
    Eigen::Isometry3d poi_mat;
    poi_mat.matrix().block<3,1>(0,3) = poi;

    bool found_ik = false;
    bool ik_in_collision = false;

    const robot_state::JointModelGroup* hand_group_ = nullptr;

    if(left_or_right == SophiaModelHandler::ARM::LEFT)
    {//Point by left hand
        hand_group_ = leftForeArmGroup_;
    }
    else
    {//Point by right hand
        hand_group_ = rightForeArmGroup_;
    }

    //First solve IK for the artificial wrist roll
    found_ik = pointing_state.setFromIK(hand_group_, poi_mat);    

    if(!found_ik)
    {
        VERBOSE_3("Pointing IK not found for ARTIFICIAL wrist roll setting.");
        return false;
    }
    else
    {
        //Check if we could set the true wrist roll
        if(handleWristRollJointOffset(pointing_state,left_or_right))
        {
            //Again, collision check is postponed.
            return true;
        }
        else
        {
            VERBOSE_3("Pointing IK found exceeds TRUE wrist roll bounds.");
            return false;
        }
    }
}

bool SophiaModelHandler::handleWristRollJointOffset(moveit::core::RobotState& pointing_state, SophiaModelHandler::ARM left_or_right)
{
    bool can_set_wrist_roll = false;

    double wristArtificialOffset;
    double wristIndexFingerOffset;

    std::string artificial_wrist_joint_name;
    std::string true_wrist_joint_name;
    moveit::core::JointModel * true_wrist_joint_model_;    

    if(left_or_right == SophiaModelHandler::ARM::LEFT)
    {//Point by left hand
        artificial_wrist_joint_name = leftArmWristArtificialJointName;
        true_wrist_joint_name = leftArmWristTrueJointName;
        true_wrist_joint_model_ = leftArmWristTrueJointModel_;

        wristIndexFingerOffset = leftWristIndexFingerOffset;
        wristArtificialOffset = leftWristArtificialOffset;
    }
    else
    {//Point by right hand
        artificial_wrist_joint_name = rightArmWristArtificialJointName;
        true_wrist_joint_name = rightArmWristTrueJointName;
        true_wrist_joint_model_ = rightArmWristTrueJointModel_;

        wristIndexFingerOffset = rightWristIndexFingerOffset;
        wristArtificialOffset = rightWristArtificialOffset;
    }

    //Roll angle needed by the ARTIFICIAL wrist joint to point its z-axis at the given poi
    double artificial_wrist_roll_angle = pointing_state.getVariablePosition(artificial_wrist_joint_name); 
    //The rotation by the TRUE wrist roll joint needed to align the z-axis with the point
    double true_wrist_roll_needed_angle
        = artificial_wrist_roll_angle + wristArtificialOffset;
    //The rotation by the TRUE wrist roll joint needed to align the INDEX FINGER with the point
    true_wrist_roll_needed_angle
        = true_wrist_roll_needed_angle + wristIndexFingerOffset;
    //Wrap the angle
    true_wrist_roll_needed_angle = wrapAngle(true_wrist_roll_needed_angle);

    VERBOSE_3("TRUE wrist roll joint needs to be at %f.",true_wrist_roll_needed_angle);

    //Check joint bounds
    can_set_wrist_roll = true_wrist_joint_model_->satisfiesPositionBounds(&true_wrist_roll_needed_angle);
    
    if(can_set_wrist_roll)
    {
        pointing_state.setVariablePosition(
            true_wrist_joint_name,true_wrist_roll_needed_angle
        );
    }

    return can_set_wrist_roll;
}

double SophiaModelHandler::wrapAngle(double revo_in) const
{//Wrap to within -pi and pi
    double revo_out = fmod(revo_in + M_PI,2*M_PI);
    if (revo_out < 0)
        revo_out += 2*M_PI;
    return (revo_out - M_PI);
}

void SophiaModelHandler::setFullbodyToRest(moveit::core::RobotState& robot_state)const
{
    //Note that the mimic joint positions is also FORCED by these calls.
    robot_state.setVariablePositions(fullBodyWithArtiMoveGroupInterface.getNamedTargetValues(fullBodyWithArtiRestConfigName));
    setBothHandsToRest(robot_state);
}

void SophiaModelHandler::setBothHandsToRest(moveit::core::RobotState& robot_state)const
{
    auto right_hand_group_rest = rightHandMoveGroupInterface.getNamedTargetValues(rightHandRestConfigName);
    auto left_hand_group_rest = leftHandMoveGroupInterface.getNamedTargetValues(leftHandRestConfigName);
    robot_state.setVariablePositions(right_hand_group_rest);
    robot_state.setVariablePositions(left_hand_group_rest);
}

void SophiaModelHandler::setHandPointingConfiguration(moveit::core::RobotState& pointing_state, SophiaModelHandler::ARM left_or_right)
{   
    //The other side that is NOT used in pointing is assumed to have been reset to home already 
    //Note that the mimic joint positions is also FORCED by these calls.
    if(left_or_right == SophiaModelHandler::ARM::RIGHT)
    {
        //Right hand to pointing configuration
        pointing_state.setVariablePositions(rightHandMoveGroupInterface.getNamedTargetValues(rightHandPointingConfigName));
    }
    else
    {
        //Left hand to pointing configuration
        pointing_state.setVariablePositions(leftHandMoveGroupInterface.getNamedTargetValues(leftHandPointingConfigName));
    }
}

bool SophiaModelHandler::computePointingConfiguration(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, SophiaModelHandler::ARM left_or_right, bool check_collision)
{
    std::string arm_choice;
    const robot_state::JointModelGroup* arm_group_ = nullptr; 
    const robot_state::JointModelGroup* hand_group_ = nullptr; 
    if(left_or_right == SophiaModelHandler::ARM::LEFT)
    {//Point by left hand
        arm_choice = "LEFT ARM";
        arm_group_ = leftArmGroup_;
        hand_group_ = leftForeArmGroup_;
    }
    else
    {//Point by right hand
        arm_choice = "RIGHT ARM";
        arm_group_ = rightArmGroup_;
        hand_group_ = rightForeArmGroup_;
    }


    bool arm_config_for_wrist_placement_found = false;
    bool forearm_config_for_hand_orientation_found = false;
    bool pointing_configuration_collision_free = false;
    

    double elapsed_time_sec = 0;
    const clock_t begin_time = clock();

    while( !(arm_config_for_wrist_placement_found && forearm_config_for_hand_orientation_found && pointing_configuration_collision_free) 
            && (elapsed_time_sec <= pointingIKTimeout) )
    {
        //Reset the flags
        arm_config_for_wrist_placement_found = false;
        forearm_config_for_hand_orientation_found = false;
        pointing_configuration_collision_free = false;

        //Reset the robot state to default all zero state,
        //that is, both arm down - this removes unecessary arm holding,
        //assuming that sophia will be doing pointing and pointing alone by one arm.
        setFullbodyToRest(pointing_state);

        //Pointing workflow
        //Compute arm configuration to place wrist
        arm_config_for_wrist_placement_found = computeArmPointingConfiguration(poi, pointing_state, left_or_right);
        if(arm_config_for_wrist_placement_found)
        {
            //Wrist and hand orientation is done after AFTER wrist has been placed reasonably
            forearm_config_for_hand_orientation_found = computeForeArmPointingConfiguration(poi, pointing_state, left_or_right);

            if(forearm_config_for_hand_orientation_found)
            {
                //If orientation is successful, then also set the target 
                //hand config to be what looks like pointing
                setHandPointingConfiguration(pointing_state,left_or_right);


                //After setting hand config to pointing like, check collision.
                if(check_collision)
                {
                    //If a kinematically valid hand pointing configuration is also found, then we do self-collision check
                    pointing_configuration_collision_free = !robotStateCollisionCheck(pointing_state);
                }
                else
                {
                    pointing_configuration_collision_free = true;
                }
            }
        }

        //Keep track of time
        elapsed_time_sec = double( clock () - begin_time ) /  CLOCKS_PER_SEC;
    }

    if( arm_config_for_wrist_placement_found 
        && forearm_config_for_hand_orientation_found 
        && pointing_configuration_collision_free)
    {//Successfully found a pointing config for the current arm

        VERBOSE_2("[%s] Found a viable pointing pose in %f seconds.",arm_choice.c_str(), elapsed_time_sec);
    }
    else
    {//Failed to find a pointing config for the current arm
        VERBOSE_2("[%s] Failed to find a pointing pose in %f seconds.",arm_choice.c_str(), elapsed_time_sec);
        if(!arm_config_for_wrist_placement_found)
        {
            VERBOSE_3("[%s] Wrist placement not found.",arm_choice.c_str());
        }
        else
        {
            if(!forearm_config_for_hand_orientation_found)
            {
                VERBOSE_3("[%s] Hand orientation not found.",arm_choice.c_str());
            }
            else
            {
                if(!pointing_configuration_collision_free)
                {
                    VERBOSE_2("[%s] Computed pointing config is in collision.",arm_choice.c_str());
                }
            }
        }
    }

    return (arm_config_for_wrist_placement_found 
            && forearm_config_for_hand_orientation_found 
            && pointing_configuration_collision_free);
}

bool SophiaModelHandler::computePointingConfigurationBothArm(const Eigen::Vector3d& poi, moveit::core::RobotState& pointing_state, bool check_collision)
{
    bool pointing_pose_found_right = false;
    bool pointing_pose_found_left = false;

    //Try right arm first
    pointing_pose_found_right = computePointingConfiguration(poi, pointing_state, SophiaModelHandler::ARM::RIGHT,check_collision);
    if(!pointing_pose_found_right)
    {//Try left arm if right arm failed
        pointing_pose_found_left = computePointingConfiguration(poi,pointing_state, SophiaModelHandler::ARM::LEFT,check_collision);
    }



    return (pointing_pose_found_right || pointing_pose_found_left);
}

bool SophiaModelHandler::planPathToGivenConfiguration(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state, 
    const robot_state::JointModelGroup* planning_group_, 
    robot_trajectory::RobotTrajectoryPtr& robot_traj_)
{

    //Construct a scene using the start robot state
    planning_scene::PlanningScenePtr start_scene_(new planning_scene::PlanningScene(currentRobotModel_));
    start_scene_->setCurrentState(start_state);

    // Now, setup the joint configuration goal as the pointing state
    planning_interface::MotionPlanRequest req;
    req.group_name = planning_group_->getName();
    moveit::core::robotStateToRobotStateMsg(start_state,req.start_state);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, planning_group_);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Call the planner
    /* Construct the planning context */
    planning_interface::MotionPlanResponse res;
    planning_interface::PlanningContextPtr context = plannerInstance->getPlanningContext(start_scene_, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        VERBOSE_1("Could not compute a plan to reach the pointing configuration.");
        return false;
    }
    else
    {
        VERBOSE_2("Successfully found a plan to reach the pointing configuration.");
    }

    //If reached here the planning is successful, then we extract the trajectory (still a path at this moment as it is not time-parameterized)
    //First get response message
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    //Then the trajectory object
    robot_traj_
        = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(currentRobotModel_,planning_group_));
    //Then copy the planned path into this trajectory
    robot_traj_->setRobotTrajectoryMsg(start_state,response.trajectory);

    return true;

}

bool SophiaModelHandler::visualizePlannedTrajectory(const robot_state::JointModelGroup* planning_group_, const robot_trajectory::RobotTrajectoryPtr& robot_traj_)
{
    //Check if the planning group name agrees with that stored in the trajectory object
    if((planning_group_->getName()).compare(robot_traj_->getGroupName()) == 0)
    {
        removeAllVisualMarkers();

        //Play the arm trajectory animation in rviz
        moveit_msgs::MotionPlanResponse response;
        robot_traj_->getRobotTrajectoryMsg(response.trajectory);

        moveit_msgs::DisplayTrajectory trajectory_msg_for_display;
        trajectory_msg_for_display.trajectory_start = response.trajectory_start;
        trajectory_msg_for_display.trajectory.push_back(response.trajectory);
        moveitVisualTools_->publishTrajectoryLine(trajectory_msg_for_display.trajectory.back(), planning_group_);
        moveitVisualTools_->trigger();
        displayPlannedTrajPublisher.publish(trajectory_msg_for_display);

        return true;
    }
    else
    {
        ROS_ERROR("Trajectory group and joint group name don't agree.");
        return false;
    }

}

bool SophiaModelHandler::executePlannedTrajectory(const robot_state::JointModelGroup* planning_group_, const robot_trajectory::RobotTrajectoryPtr& robot_traj_)
{
    //Check if the planning group name agrees with that stored in the trajectory object
    if((planning_group_->getName()).compare(robot_traj_->getGroupName()) == 0)
    {
        //To execute the trajectory, interpolate and publish the parameterized path 
        //of the WHOLE robot body regardeless of group name to hrsdk at given interval
        double time_on_traj_now = 0;
        double total_dur = robot_traj_->getDuration();
        while(time_on_traj_now <= total_dur)
        {
            //Update the global robot state of the class and publish the waypoint
            publishPlannedPathWayPointToHRSDK(
                time_on_traj_now,
                currentRobotState_,
                robot_traj_
            );

            //Visualize the latest state in tviz.
            visualizeRobotStateInRviz(*currentRobotState_);

            ros::Duration(publish_interval_sec).sleep();

            time_on_traj_now = time_on_traj_now + publish_interval_sec;
        }

        // //Hold the end position for debugging
        // while(true)
        // {
        //     publishPlannedPathWayPointToHRSDK(
        //         time_on_traj_now,
        //         currentRobotState_,
        //         robot_traj_
        //     );
        //     ros::Duration(publish_interval_sec).sleep();
        // }

        return true;
    }
    else
    {
        ROS_ERROR("Trajectory group and joint group name don't agree.");
        return false;
    }
}

bool SophiaModelHandler::parametrizePlannedPath(double dur,const robot_state::JointModelGroup* planning_group_, robot_trajectory::RobotTrajectoryPtr& robot_traj_) const
{
    if((planning_group_->getName()).compare(robot_traj_->getGroupName()) == 0)
    {
        //Parameterization method
        trajectory_processing::IterativeSplineParameterization isp (true);//Stamp UNIT: Seconds!!!!
        
        // Joint limits are enforced when doing the time stamping according to what is specified in joint_limits.yaml in the moveit configuration pacakge.
        isp.computeTimeStamps(*robot_traj_,maxSpeedScaleFactor,maxAccelarationScaleFactor);

        //Do a rescaling
        int num_way_points = robot_traj_->getWayPointCount();
        double scaling_ratio = dur/(robot_traj_->getWayPointDurationFromStart(num_way_points-1));
        for(int i = 0 ; i < num_way_points; i++)
        {
            double original_interval = robot_traj_->getWayPointDurationFromPrevious(i);
            double new_interval = original_interval * scaling_ratio;
            
            robot_traj_->setWayPointDurationFromPrevious(i,new_interval);
        }

        return true;
    }
    else
    {
        ROS_ERROR("Trajectory group and joint group name don't agree.");
        return false;
    }
}

void SophiaModelHandler::publishPlannedPathWayPointToHRSDK(double time_instant,	robot_state::RobotStatePtr& state_to_update_, const robot_trajectory::RobotTrajectoryPtr& robot_traj_) const
{
    //Use the built in interpolation to update the robot state to what it should be at the given time instant
    robot_traj_->getStateAtDurationFromStart(
        std::min(time_instant,robot_traj_->getDuration()),//Avoid going beyond bound
        state_to_update_);

    //Publish this state to hrsdk
    publishRobotStateToHRSDK(*state_to_update_,robot_traj_->getGroup());
}

void SophiaModelHandler::publishRobotStateToHRSDK(const robot_state::RobotState& state_to_pub, const moveit::core::JointModelGroup* group_)const
{
    //Retrieve joint value of interests (we trust the two lists are MANUALLY WELL FORMED of 1-1 correspondence)
    std::vector<double> joint_values_for_hrsdk = std::vector<double>();
    std::vector<std::string> joint_names_for_hrsdk = std::vector<std::string>();
    for(int i = 0 ; i < armStateJointNamesInURDF.size(); i++)
    {
        //Check if this name is in the group
        if(group_->hasJointModel(armStateJointNamesInURDF[i]))
        {
            //Retrieve value from the robot state
            double urdf_value = state_to_pub.getVariablePosition(armStateJointNamesInURDF[i]);
            //Tweak by directional change and offset and publish to hrsdk
            joint_values_for_hrsdk.push_back(
                (urdf_value + armStateOffsetTweak[i]) * armStateDirectionalTweak[i]
            );
            //Record the corresponding joint name in hrsdk
            joint_names_for_hrsdk.push_back(
                armStateJointNamesInBlender[i]
            );
        }
    }

    //Publish waypoint values to hrsdk
    publishJointValuesToHRSDK(joint_values_for_hrsdk,joint_names_for_hrsdk);
}

void SophiaModelHandler::publishJointValuesToHRSDK(const std::vector<double>& joint_values,const std::vector<std::string>& joint_names)const
{
    //Form an hr_msg
    hr_msgs::TargetPosture posture_msg;
    posture_msg.names = joint_names;
    posture_msg.values = joint_values;

    //Publish
    hrArmStatePublisher.publish(posture_msg);
}

void SophiaModelHandler::testPointing()
{
    //First reset initial robot state
    setFullbodyToRest(*currentRobotState_);
    visualizeRobotStateInRviz(*currentRobotState_);

    //Distribution for generating random points in space
    bool check_collision = true;
    double point_sample_mean = 0;
    double point_sample_std = 0.6;
    std::normal_distribution<double> distribution_poi(
        point_sample_mean,
        point_sample_std);
    Eigen::Isometry3d center_transform = currentRobotState_->getGlobalLinkTransform("shoulder_center");


    //Some extra deuggging trinkets for planning interface co debugging
    int succ_cnt = 0;

    while(true)
    {
        //The place to point (point of interest):
        Eigen::Vector3d poi = center_transform.matrix().block<3,1>(0,3);
        poi(0) = distribution_poi(rng_generator);
        poi(1) = distribution_poi(rng_generator);
        poi(2) =  distribution_poi(rng_generator);

        //Visualize the point of interest
        placeSphereRvizMarker(
            poi,
            rviz_visual_tools::GREEN,
            rviz_visual_tools::LARGE);

        //Make a copy of the cuurent robot state
        moveit::core::RobotState pointing_state = *currentRobotState_;
        
        //Compute pointing configuration
        bool pointing_pose_found = computePointingConfigurationBothArm(poi,pointing_state,check_collision);
        if(pointing_pose_found)
        {
            //Compute a path to reach the pointing configuration found 
            //from the current state for the ENTIRE body
            //so that all joints get properly interpolated.
            robot_trajectory::RobotTrajectoryPtr robot_traj_ = nullptr;
            bool plan_found = false;
            bool group_found = false;

            //To prevent unnecessary movement we do wish to suppress planning for the unmoved
            //arm by checking the distance between the hand config, since if this arm is going 
            //from rest to pointing or the other way around, the hand config would change significantly
            const robot_state::JointModelGroup* planning_group_ = nullptr;
            group_found = choosePlanningGroupSuppressExcessiveArmMotion(*currentRobotState_,pointing_state,planning_group_);
            
            if(group_found)
            {
                //Do planning with the selected group
                plan_found = planPathToGivenConfiguration(*currentRobotState_, pointing_state,planning_group_, robot_traj_);
                //Post planning processing
                if(plan_found)
                {
                    //Parameterize the path found given desired duration
                    parametrizePlannedPath(2,planning_group_, robot_traj_);

                    // //Visualize and execute the planned trajectory
                    removeAllVisualMarkers();
                    // visualizePlannedTrajectory(planning_group_, robot_traj_);

                    //Execute by publishing to hrsdk
                    executePlannedTrajectory(planning_group_, robot_traj_);

                    //Connect hand origins and the poi
                    //At this moment one of the arm should has been moved
                    //to the pointing configuration
                    placeLineRvizMarker(
                        poi,
                        (currentRobotState_->getGlobalLinkTransform(leftArmEELinkName)).matrix().block<3,1>(0,3),
                        rviz_visual_tools::YELLOW,
                        rviz_visual_tools::LARGE);
                    placeLineRvizMarker(
                        poi,
                        (currentRobotState_->getGlobalLinkTransform(rightArmEELinkName)).matrix().block<3,1>(0,3),
                        rviz_visual_tools::BLUE,
                        rviz_visual_tools::LARGE);   


                    //Publish this successful frame to tf for later co-debugging
                    if(succ_cnt <= 2)
                    {
                        geometry_msgs::TransformStamped msg;
                        msg.header.stamp = ros::Time::now();
                        msg.header.frame_id = world_frame_name;
                        msg.child_frame_id = test_frame_names[succ_cnt];

                        msg.transform.rotation.w = 1;
                        msg.transform.rotation.y = 0;
                        msg.transform.rotation.z = 0;
                        msg.transform.rotation.x = 0;

                        msg.transform.translation.x = poi[0];
                        msg.transform.translation.y = poi[1];
                        msg.transform.translation.z = poi[2];

                        tf2_broadcaster.sendTransform(msg);

                        succ_cnt ++;
                    }                   
                }
            }
        }

        VERBOSE_1("Press the key to keep testing (Y) or stop testing (N).");
        std::string in;
        std::cin >> in;
        if(in.compare("Y") == 0 || in.compare("y") == 0)
        {
            VERBOSE_1("Continue pointing test.");
            continue;
        }
        else
        {
            VERBOSE_1("Stop pointing test..");
            break;
        }
    }

}

void SophiaModelHandler::testMappingDir()
{
    removeAllVisualMarkers();
    
    //Ignore limit for testing purpose
    ros::param::set(hrArmIgnoreLimitTopicName,true);

    for(int i = 14 ; i < armStateJointNamesInURDF.size(); i++)
    {
        std::string joint_name_urdf = armStateJointNamesInURDF[i];
        std::vector<std::string> joint_name_hr = {armStateJointNamesInBlender[i]};        
        
        VERBOSE_1("Testing joint #%d, named %s in urdf and %s in hrsdk.",
            i,
            joint_name_urdf.c_str(),
            joint_name_hr[0].c_str());

        while(true)
        {
            //Loop frop -pi to pi
            double step = (2*M_PI)/180;
            double interval_sec = 0.05;            
            double joint_val = -M_PI;

            setFullbodyToRest(*currentRobotState_);
            
            while(joint_val <= M_PI)
            {
                std::vector<double> joint_val_shared = 
                    {(joint_val + armStateOffsetTweak[i])*armStateDirectionalTweak[i]};

                //Publish to hr
                publishJointValuesToHRSDK(joint_val_shared,joint_name_hr);

                //Visualize by urdf model in rviz
                currentRobotState_->setVariablePosition(joint_name_urdf,joint_val);
                // currentRobotState_->enforceBounds();
                visualizeRobotStateInRviz(*currentRobotState_);

                //Increment and sleep
                joint_val = joint_val + step;
                ros::Duration(interval_sec).sleep();
            }
            
            //Check if we keep testing the current joint
            VERBOSE_1("Press the key to keep testing this joint (Y) or move on (N).");
            std::string in;
            std::cin >> in;
            if(in.compare("Y") == 0 || in.compare("y") == 0)
            {
                VERBOSE_1("Keep testing THIS joint.");
                continue;
            }
            else
            {
                VERBOSE_1("Move on to the NEXT joint.");
                break;
            }
        }

        
    }
}

void SophiaModelHandler::testMappingOffset()
{
    removeAllVisualMarkers();
    
    //Ignore limit for testing purpose
    ros::param::set(hrArmIgnoreLimitTopicName,true);

    //Initialize this vector to default pose values
    setFullbodyToRest(*currentRobotState_);

    //Record an initial pose value
    std::vector<double> joint_val_shared_full_urdf = std::vector<double>();  
    std::vector<double> joint_val_shared_full_hr = std::vector<double>();  
    for(int i = 0 ; i < armStateJointNamesInURDF.size(); i++)
    {
        //Retrieve value from the robot state
        joint_val_shared_full_urdf.push_back(
            currentRobotState_->getVariablePosition(armStateJointNamesInURDF[i])
        );
        //Offset to get values in hr
        joint_val_shared_full_hr.push_back(
          (joint_val_shared_full_urdf[i] + armStateOffsetTweak[i])*armStateDirectionalTweak[i]  
        );
    }

    //Set a pose for testing
    for(int i = 0 ; i < armStateJointNamesInURDF.size(); i++)
    {        
        VERBOSE_1("Setting joint #%d, named %s in urdf and %s in hrsdk.",
            i,
            armStateJointNamesInURDF[i].c_str(),
            armStateJointNamesInBlender[i].c_str());

        //Loop over the range of this joint, starting from 0 rad joint pos in urdf
        double step = (10*M_PI)/180;
        joint_val_shared_full_urdf[i] = 0;
        //Set this joint
        while(joint_val_shared_full_urdf[i] <= 50*M_PI)
        {
            //Update the value
            joint_val_shared_full_hr[i] = (joint_val_shared_full_urdf[i] + armStateOffsetTweak[i])*armStateDirectionalTweak[i];

            //Publish to hr
            publishJointValuesToHRSDK(joint_val_shared_full_hr,armStateJointNamesInBlender);  
            //Visualize by urdf model in rviz
            currentRobotState_->setVariablePositions(armStateJointNamesInURDF,joint_val_shared_full_urdf);
            // currentRobotState_->enforceBounds();
            visualizeRobotStateInRviz(*currentRobotState_);

            //Check if we keep increseasing the current joint's value
            VERBOSE_1("Current joint angle is %f. Is this joint position good for testing?",joint_val_shared_full_urdf[i]);
            std::string in;
            std::cin >> in;
            if(in.compare("N") == 0 || in.compare("n") == 0)
            {
                VERBOSE_1("Keep modifying THIS joint.");
            }
            else
            {
                VERBOSE_1("Move on to the NEXT joint.");
                break;
            }
            
            //Increment and ask
            joint_val_shared_full_urdf[i] = joint_val_shared_full_urdf[i] + step;
        }
        //Check if further modification is needed
        VERBOSE_1("Keep modifying the next joint?");
        std::string in;
        std::cin >> in;           
        if(in.compare("Y") == 0 || in.compare("y") == 0)
        {
            VERBOSE_1("Keep modifying.");
            continue;
        }
        else
        {
            VERBOSE_1("Move on to observation stage.");
            break;
        }
    }    

    //Those untouched joints would have their default state values set.
    double interval_sec = 0.1;            
    while(true)
    {
        //Publish to hr
        visualizeRobotStateInRviz(*currentRobotState_);
        //Update in ros
        currentRobotState_->setVariablePositions (armStateJointNamesInURDF,joint_val_shared_full_urdf);
        publishJointValuesToHRSDK(joint_val_shared_full_hr,armStateJointNamesInBlender);
        //Sleep a while
        ros::Duration(interval_sec).sleep();
    }

}

void SophiaModelHandler::executeTrajectoryWrapper(robot_trajectory::RobotTrajectoryPtr& robot_traj_)
{
    executePlannedTrajectory(robot_traj_->getGroup(),robot_traj_);
}

bool SophiaModelHandler::generatePointingConfiguration(
    const geometry_msgs::TransformStamped& poi, 
    moveit::core::RobotStatePtr& robot_state_)
{
    robot_state_ = moveit::core::RobotStatePtr(
        new moveit::core::RobotState(*currentRobotState_)
    );

    Eigen::Vector3d poi_eigen;
    poi_eigen[0] = poi.transform.translation.x;
    poi_eigen[1] = poi.transform.translation.y;
    poi_eigen[2] = poi.transform.translation.z;

    bool config_found = computePointingConfigurationBothArm(
        poi_eigen,
        *robot_state_,//copy pointing configuration to this state
        true//check collision
    );

    // //Debugging
    // placeSphereRvizMarker(poi_eigen, rviz_visual_tools::colors::BLUE, rviz_visual_tools::scales::XLARGE);


    return config_found;
}

bool SophiaModelHandler::generateAddressingConfiguration(
    const geometry_msgs::TransformStamped& poi, 
    moveit::core::RobotStatePtr& robot_state_)
{
    robot_state_ = moveit::core::RobotStatePtr(
        new moveit::core::RobotState(*currentRobotState_)
    );

    Eigen::Vector3d poi_eigen;
    poi_eigen[0] = poi.transform.translation.x;
    poi_eigen[1] = poi.transform.translation.y;
    poi_eigen[2] = poi.transform.translation.z;

    //Addressing configuration is just pointing with a different hand configuration
    //get pointing configuration
    bool config_found = computePointingConfigurationBothArm(
        poi_eigen,
        *robot_state_,//copy pointing configuration to this state
        true//check collision
    );
    if(config_found)
    {
        // //Force hand to be at rest config so its more like addressing.
        // //Assumption here is that this would NOT cause extra collisions
        setBothHandsToRest(*robot_state_);   
    }
    
    
    // //Debugging
    // placeSphereRvizMarker(poi_eigen, rviz_visual_tools::colors::BLUE, rviz_visual_tools::scales::XLARGE);
    
    return config_found;    
}

bool SophiaModelHandler::planTrajectoryToTargetConfiguration(
    moveit::core::RobotStateConstPtr& start_state_,
    moveit::core::RobotStateConstPtr& goal_state_, 
    const double duration,
    robot_trajectory::RobotTrajectoryPtr& robot_traj_)
{

    const robot_state::JointModelGroup* planning_group_ = nullptr;
    bool group_found = choosePlanningGroupSuppressExcessiveArmMotion(*start_state_,*goal_state_,planning_group_);
    if(group_found)
    {
        robot_traj_ = nullptr;
        bool plan_found = planPathToGivenConfiguration(*start_state_,*goal_state_, planning_group_, robot_traj_);

        if(plan_found)
        {
            parametrizePlannedPath(duration,planning_group_,robot_traj_);
        }

        return plan_found;
    }
    else
    {
        VERBOSE_2("No approriate planning group found. Returning.");
        return false;
    }
}

moveit::core::RobotStatePtr SophiaModelHandler::getRestRobotStateCopy()
{
    moveit::core::RobotStatePtr robot_state_ = moveit::core::RobotStatePtr(
        new moveit::core::RobotState(*currentRobotState_)
    );
    setFullbodyToRest(*robot_state_);
    return robot_state_;
}

moveit::core::RobotStatePtr SophiaModelHandler::getCurrentRobotStateCopy()const
{
    moveit::core::RobotStatePtr robot_state_ = moveit::core::RobotStatePtr(
        new moveit::core::RobotState(*currentRobotState_)
    );
    return robot_state_;
}

bool SophiaModelHandler::choosePlanningGroupSuppressExcessiveArmMotion(
    const moveit::core::RobotState& start_state, 
    const moveit::core::RobotState& goal_state,
    const robot_state::JointModelGroup*& planning_group_)const
{
    //Prevent excessive joint movement arising from planning by removing the part not moving from the planning group.
    planning_group_ = nullptr;
    double right_arm_group_dist = start_state.distance(goal_state,rightArmGroup_);
    double left_arm_group_dist = start_state.distance(goal_state,leftArmGroup_);

    // ROS_WARN("Press any key to visualize start state.");
    // std::cin.get();
    // visualizeRobotStateInRviz(start_state);

    // ROS_WARN("Press any key to visualize goal state.");
    // std::cin.get();
    // visualizeRobotStateInRviz(goal_state);

    // ROS_WARN("Press any key to check suppress.");
    // std::cin.get();

    VERBOSE_4("Config change in arm group: left %f, right %f, threshold %f.",
        left_arm_group_dist,right_arm_group_dist,suppressDistPlanningThreshold);

    if( right_arm_group_dist >= suppressDistPlanningThreshold && left_arm_group_dist <= suppressDistPlanningThreshold )
    {//Right hand is moving, left is not
        planning_group_ = fullBodyGroupNoArtiNoLeft_;
        // ROS_WARN("Suppressing left arm group.");
    }
    else if( right_arm_group_dist <= suppressDistPlanningThreshold && left_arm_group_dist >= suppressDistPlanningThreshold )
    {
        planning_group_ = fullBodyGroupNoArtiNoRight_;
        // ROS_WARN("Suppressing right arm group.");
    }
    else if(right_arm_group_dist >= suppressDistPlanningThreshold && left_arm_group_dist >= suppressDistPlanningThreshold )
    {
        planning_group_ = fullBodyGroupNoArti_;
        // ROS_WARN("Do not suppress.");
    }
    else
    {
        ROS_WARN("Both hand group has no config change!");
        //Do not change planning group, return false.
        return false;
    }

    return true;
}

std::string SophiaModelHandler::getBaseJointName()const
{
    return currentRobotModel_->getRootJoint()->getChildLinkModel()->getName();
}

void SophiaModelHandler::rectifySophiaPoseThread()const
{//This thread will force sophia into a rest pose, thereby suppressing the keep-alive pose that is on
 //all the time, allowing a more instant transition into our designated cuing actions
    
    moveit::core::RobotState rest_state = *currentRobotState_;
    setFullbodyToRest(rest_state);
    while(keep_forcing)
    {
        publishRobotStateToHRSDK(rest_state,fullBodyGroupNoArti_);
        ros::Duration(publish_interval_sec).sleep();
    }
}

void SophiaModelHandler::rectifySophiaPoseToRest()
{
    keep_forcing = true;
    forceSophiaRestThread_ = new std::thread(&SophiaModelHandler::rectifySophiaPoseThread,this);
}

void SophiaModelHandler::stopRectifyingSophiaPoseToRest()
{
    keep_forcing = false;
}










