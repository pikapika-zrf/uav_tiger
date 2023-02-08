
#include "off_mission_node.h"

TaskManager::TaskManager()
{
    /* subscriptions */
    m_rcin_sub_ = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10,&TaskManager::rcinCallback,this);
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &TaskManager::state_cb,this);
    response_sub  = nh.subscribe
            ("navigator/vehicle_action_response", 5,&TaskManager::response_cb,this);
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &TaskManager::local_pose_cb,this);
    online_target_sub_ = nh.subscribe
            ("online_target", 10, &TaskManager::onlineTarget_cb, this);  // online target produced by other node, such as vision tracking, obstacle avoidence.

    m_global_pos_sub_ = nh.subscribe("mavros/global_position/global", 10, &TaskManager::globalPosition_cb, this);

    /* publications */
    action_pub = nh.advertise<navigator::action>("navigator/vehicle_action",2);
    pos_nav_pub = nh.advertise<geometry_msgs::PoseStamped>("navigator/pos_nav",2);

    /* services */
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    offb_set_mode.request.custom_mode = "OFFBOARD"; // AUTO.LAND
    offboard_sent_flag_ = false ;
    last_request = ros::Time::now();

    /*params*/
    ros::NodeHandle private_nh("~");
    private_nh.param("takeoff_hight", takeoff_height, 1.0);
    private_nh.param("cruise_hight", cruise_height_, 5.0);
    private_nh.param("waypoint_width", waypoint_width_, 4.0);
    private_nh.param("outdoor_indoor_flag", m_outdoor_indoor_flag_, 0);

    private_nh.param("nav_acc_radius", _nav_acc_radius, 0.5);
    private_nh.param("nav_acc_angle", _nav_acc_angle, 0.1);
    private_nh.param("uav_id", uav_id, 1);
    private_nh.param("local_global_flag", local_global_flag_, 1);

    // load yaml files for offline waypoints

    private_nh.getParam("waypoints",wp_list);
    initLocalWaypoints(wp_list);

    XmlRpc::XmlRpcValue gps_matrix;
    private_nh.getParam("gps_waypoints", gps_matrix);
    initGlobalWaypoint(gps_matrix);

    resetOffMission();
}

TaskManager::~TaskManager()
{

}

/**
 * @brief TaskManager::mainLoop
 * Main entry point for the code
 */
void TaskManager::mainLoop()
{
    // If first time arm, reset whole mission
    checkArmingState();

    // Set relative origin, taking local position drift into account
    if(!ground_origin_position_initialized_flag_ )
    {
        processGroundOrigin();
        return;
    }

    setOffboardMode();

    // convert global gps waypoints to local enu waypoints;
    if( local_global_flag_ == 0 && !gps_pos_enu_set_flag_)
    {
        getGlobalEnuWaypoints();
        if ( gps_pos_enu_set_flag_)
        {
            waypoints.clear();
            // 1. generate waypoints from interest points;
            convertWindowPosToWaypoint();
            // 2. use all gps converted waypoints
            // waypoints = gps_enu_pos_;
            printWaypoints();
            ROS_INFO("local_global_flag_ == %d, switching Global gps Waypoints -> local enu waypoints", local_global_flag_);
        }
    }

    /******** IDLE, TAKEOFF, HOVER, PATH, LAND ************/

    // Case IDLE: while no OFFBOARD RC INPUT
    if (!isOffboardSwitchOn(m_rcin_) || cur_response.finished_behavior == LAND) 
    {   
        if ( setBehaviorIDLE() )     
            offboard_sent_flag_ = false;
    }    
    
    // Case TAKEOFF & HOVER: First Time is TAKEOFF, the rest is HOVER
    if ( isOffboardSwitchOn(m_rcin_) && isSwitchTakeoffHover(m_rcin_))
    {
      if (!takeoff_rc_flag) 
      {
        if ( setBehaviorTAKEOFF(takeoff_height) ) 
        {
          takeoff_rc_flag = true;
        }
      }
      if (cur_response.finished_behavior == TAKEOFF && !takeoff_finished_flag) 
      {
        takeoff_finished_flag = true;
      }
      if (takeoff_finished_flag && takeoff_rc_flag) 
      {
        setBehaviorHOVER();
      }
    }

    // Case: PATH: online or offline waypoint
    if( isOffboardSwitchOn(m_rcin_) && isSwitchPath(m_rcin_) && takeoff_rc_flag)
    {  
        if ( !all_waypoint_reached_flag_ )
        {
            if( setBehaviorPATH() )
            {
            	takeoff_finished_flag = true; // Once switched to PATH, change TAKEOFF to HOVER
            }
            handle_waypoint_flight();
        }
        else // in PATH switch and all waypoint reached, land
        {
            if ( setBehaviorLAND() )
            {
                ROS_INFO("All waypoints reached, land at current point");
            }
        }
    }

    // Case: LAND
    if ( isOffboardSwitchOn(m_rcin_) && isSwitchLand(m_rcin_) && takeoff_rc_flag)
    {
        if ( setBehaviorLAND() )
        {
          takeoff_finished_flag = true; // enable hover once go to land position once
          offboard_sent_flag_   = false; // indicate offboard not sent 
        }
    }
    
}

/**
 * @brief TaskManager::checkArmingState
 * if first time to arm, will reset whole mission.
 */
void TaskManager::checkArmingState()
{
    if(!current_state.armed && !reset_off_mission_flag_)
    {
        reset_off_mission_flag_ = true;
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("Disarmed! Will restart the mission next time armed!\n");
    }
    if(current_state.armed && reset_off_mission_flag_)
    {
        reset_off_mission_flag_ = false;
        ROS_INFO("First time to Arm, Mission Reset !!!");
        resetOffMission();
    }
}

/**
 * @brief TaskManager::resetOffMission
 *  Reset whole mission.
 */
void TaskManager::resetOffMission()
{

  _current_wpindex = 0;
  m_online_offline_flag_ = 0;
  current_waypoint_published_flag_ = false;
  takeoff_finished_flag = false;
  takeoff_rc_flag = false;

  all_waypoint_reached_flag_ = false;
  resetOnGroundOrigin();

  memset(gpsorigin, 0, sizeof(gpsorigin));
  gps_origin_counter_ = 0;
  gps_pos_enu_set_flag_ = false;
}

void TaskManager::initLocalWaypoints(XmlRpc::XmlRpcValue &wp_list)
{
    waypoints.clear();
    geometry_msgs::PoseStamped tempPose;

    for (size_t i = 0; i < wp_list.size(); i++)
    {
        tempPose.header.seq = i;
        XmlRpc::XmlRpcValue data_list(wp_list[i]);

        // get position
        tempPose.pose.position.x = data_list[0];
        tempPose.pose.position.y = data_list[1];
        tempPose.pose.position.z = data_list[2];

        // get orientation
        tf::Quaternion q = tf::createQuaternionFromYaw(data_list[3]);

        tf::quaternionTFToMsg(q, tempPose.pose.orientation);
        waypoints.push_back(tempPose);
    }

    ROS_INFO("Loaded enu local waypoint size is %d ", (int)waypoints.size());
    printWaypoints();
}

/**
 * @brief TaskManager::onlineTarget_cb
 * receive online position target from other node
 * @param msg
 */
void TaskManager::onlineTarget_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    m_online_target_ = *msg;
    m_online_offline_flag_ = 1;  
}

void TaskManager::handle_waypoint_flight()
{
    
    if( m_online_offline_flag_ == 1) // online target following
    {
        float current_yaw;
        float new_online_target_x;
        float new_online_target_y;
        current_yaw = tf::getYaw(current_local_pos.pose.orientation);
        new_online_target_x = m_online_target_.pose.position.x * cos(current_yaw) - m_online_target_.pose.position.y * sin(current_yaw);
        new_online_target_y = m_online_target_.pose.position.x * sin(current_yaw) + m_online_target_.pose.position.y * cos(current_yaw);
        pose_nav.pose.position.x = current_local_pos.pose.position.x + new_online_target_x;
        pose_nav.pose.position.y = current_local_pos.pose.position.y + new_online_target_y;
        pose_nav.pose.position.z = current_local_pos.pose.position.z;
        pose_nav.pose.orientation = current_local_pos.pose.orientation;
        // pose_nav.pose.position.x = current_local_pos.pose.position.x + m_online_target_.pose.position.x;
        // pose_nav.pose.position.y = current_local_pos.pose.position.y + m_online_target_.pose.position.y;

        pos_nav_pub.publish(pose_nav);
    }
    else
    {
        if( _current_wpindex < waypoints.size() ) 
        {        
            if(!current_waypoint_published_flag_ )
            {
                pose_nav.pose.position.x = waypoints.at(_current_wpindex).pose.position.x;
                pose_nav.pose.position.y = waypoints.at(_current_wpindex).pose.position.y;
                pose_nav.pose.position.z = waypoints.at(_current_wpindex).pose.position.z;
                pose_nav.pose.orientation = waypoints.at(_current_wpindex).pose.orientation;
                
                addRefOrigin(&pose_nav);  // add origin bias for position drift
                pos_nav_pub.publish(pose_nav);
                current_waypoint_published_flag_ = true;
            }
        
            if(cur_response.finished_behavior == WAYPOINT_FLIGHT && cur_response.behavior_finished)
            {
                cur_response.behavior_finished = false;
                _current_wpindex++;
                current_waypoint_published_flag_ = false;
                ROS_INFO("_current_wpindex++ is %d ", _current_wpindex);
            }
        }
        else
        {
            all_waypoint_reached_flag_ = true; 
        }
    }   
}


void TaskManager::clear_switch_flags_except(int behavior)
{

    for(int i = 0 ; i < MAX_BEHAVIOR_NO ; i++ )
    {
        if (i != behavior)
        init_switch_flags[i] = false;
        
    }
}


void TaskManager::setOnGroundOrigin()
{
     m_ref_origin_.pose.position.x = current_local_pos.pose.position.x;
     m_ref_origin_.pose.position.y = current_local_pos.pose.position.y;
     m_ref_origin_.pose.position.z = current_local_pos.pose.position.z;
     ground_origin_position_initialized_flag_ = true;
     ROS_INFO("refernce origin set at position x  %f y  %f z %f " , m_ref_origin_.pose.position.x , 
                                                                       m_ref_origin_.pose.position.y , 
                                                                       m_ref_origin_.pose.position.z);
}

void TaskManager::resetOnGroundOrigin()
{
    ground_origin_position_initialized_flag_ = false;
    origin_counter  = 0;
    m_ref_origin_.pose.position.x = 0;
    m_ref_origin_.pose.position.y = 0;
    m_ref_origin_.pose.position.z = 0;
}

void TaskManager::addRefOrigin(geometry_msgs::PoseStamped  *ref)
{
    ref->pose.position.x = ref->pose.position.x + m_ref_origin_.pose.position.x;
    ref->pose.position.y = ref->pose.position.y + m_ref_origin_.pose.position.y;
    ref->pose.position.z = ref->pose.position.z + m_ref_origin_.pose.position.z;
}

void TaskManager::subRefOrigin(geometry_msgs::PoseStamped  *ref)
{
    ref->pose.position.x = ref->pose.position.x - m_ref_origin_.pose.position.x;
    ref->pose.position.y = ref->pose.position.y - m_ref_origin_.pose.position.y;
    ref->pose.position.z = ref->pose.position.z - m_ref_origin_.pose.position.z;
}


/**
 * @brief TaskManager::processGroundOrigin
 * in outdoor mode, waite for gps fix and set ground origin
 * in indoor mode, slam, just set.
 */
void TaskManager::processGroundOrigin()
{
    if (m_outdoor_indoor_flag_ == 0 ) // outdoor, wait for gps fix
    {
        if(m_global_pos_.status.status == sensor_msgs::NavSatStatus::STATUS_FIX)
        {
            setOnGroundOrigin();
        }
    }
    else // indoor, just set
    {
        setOnGroundOrigin();
    }
}

/********************** Behavior related functions **********************/

bool TaskManager::setBehaviorIDLE()
{
  if (!init_switch_flags[IDLE]) 
  {
    cur_action.behavior = IDLE;
    cur_action.uav_id = uav_id;
    cur_action.params.clear();
    action_pub.publish(cur_action);
    init_switch_flags[IDLE] = true;
    ROS_INFO("IT'S IDLE");
    ROS_INFO("---------------");
    clear_switch_flags_except(IDLE);
    return true;
  }
  else
    return false;
}

bool TaskManager::setBehaviorTAKEOFF(double height)
{
  if (!init_switch_flags[TAKEOFF]) 
  {
    cur_action.behavior = TAKEOFF;
    cur_action.params.clear();
    cur_action.params.push_back(height);
    action_pub.publish(cur_action);
    init_switch_flags[TAKEOFF] = true;
    ROS_INFO("IT'S TAKEOFF %f ", height);
    ROS_INFO("---------------");
    clear_switch_flags_except(TAKEOFF);
    return true;
  }
  else
    return false;
}

bool TaskManager::setBehaviorHOVER() 
{
  if (!init_switch_flags[HOVER]) 
  {
    cur_action.behavior = HOVER;
    cur_action.params.clear();
    action_pub.publish(cur_action);
    init_switch_flags[HOVER] = true;
    ROS_INFO("IT'S HOVER");
    ROS_INFO("---------------");
    clear_switch_flags_except(HOVER);
    return true;
  }
  else 
    return false;
}

bool TaskManager::setBehaviorLAND() 
{
  if (!init_switch_flags[LAND]) 
  {
    cur_action.behavior = LAND;
    cur_action.params.clear();
    action_pub.publish(cur_action);
    init_switch_flags[LAND] = true;
    ROS_INFO("IT'S LAND");
    ROS_INFO("---------------");
    clear_switch_flags_except(LAND);
    return true;
  }
  else
    return false;
}

bool TaskManager::setBehaviorPATH() 
{
  if (!init_switch_flags[WAYPOINT_FLIGHT]) 
  {
    cur_action.behavior = WAYPOINT_FLIGHT;
    cur_action.params.clear();
    cur_action.onlineflag = m_online_offline_flag_;
    cur_action.params.push_back(_nav_acc_radius);
    cur_action.params.push_back(_nav_acc_angle);
    current_waypoint_published_flag_ = false;
    //initTagetVector(wp_list);
    action_pub.publish(cur_action);
    init_switch_flags[WAYPOINT_FLIGHT] = true;
    clear_switch_flags_except(WAYPOINT_FLIGHT);

    ROS_INFO("IT'S WAYPOINT_FLIGHT");
    ROS_INFO("Current waypoint index %d", _current_wpindex);
    ROS_INFO("---------------");

    return true;
  }
  else
    return false;
}

/**
 * @brief TaskManager::setOffboardMode
 * set offboard in any of the two conditions
 * 1. first enter offboard by rc;
 * 2. switch from AUTO.LAND to Offboard while landing
 */
void TaskManager::setOffboardMode()
{
    if ( checkEnterOffboardByRcSwitch() ||
            (isOffboardSwitchOn(m_rcin_) && !isSwitchLand(m_rcin_) && !offboard_sent_flag_ ) )
    {
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
       {
          ROS_INFO("First Time Offboard enabled by SetMode");
          offboard_sent_flag_ = true;
       }
       last_request = ros::Time::now();            
    }

    if (current_state.mode != "OFFBOARD" 
            && ros::Time::now() - last_request > ros::Duration(5.0) 
            && !isSwitchLand(m_rcin_)
            && isOffboardSwitchOn(m_rcin_)
            && !all_waypoint_reached_flag_)
    {
       if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
             ROS_INFO("Offboard enabled by SetMode");
        }
        last_request = ros::Time::now();
    }
}

/********************** Behavior related functions **********************/

void TaskManager::getGlobalEnuWaypoints(){
    if (gps_origin_counter_<20)
    {
        gps_origin_counter_++;
        gpsorigin[0] += m_global_pos_.latitude;
        gpsorigin[1] += m_global_pos_.longitude;
        gpsorigin[2] += m_global_pos_.altitude;
    }
    else if (gps_origin_counter_==20)
    {
        ROS_INFO("-- converting global gps waypoints to enu --- ");

        gpsorigin[0] = gpsorigin[0]/20;
        gpsorigin[1] = gpsorigin[1]/20;
        gpsorigin[2] = gpsorigin[2]/20;

        struct map_projection_reference_s ref;
        map_projection_init(&ref, gpsorigin[0], gpsorigin[1]);

        float x, y, z; //NED
        gps_enu_pos_.clear();
        //convert gps position to ENU frame wst to home gps origin coordinate
        for (int i=0; i < gps_pos_matrix_.size(); i++)
        {
            if (map_projection_project(&ref, gps_pos_matrix_.at(i).x, gps_pos_matrix_.at(i).y, &x, &y)==0)
            {
                // NED hight
                z = gpsorigin[2] - gps_pos_matrix_.at(i).z;
                convertNED2ENU(&x, &y, &z);

                geometry_msgs::PoseStamped  tmp;
                // ENU
                //REFERENCE tmp;
                tmp.pose.position.x = x;
                tmp.pose.position.y = y;
                tmp.pose.position.z = z;

                // get orientation
                tf::Quaternion q = tf::createQuaternionFromYaw( gps_pos_matrix_.at(i).c );

                tf::quaternionTFToMsg(q, tmp.pose.orientation);
                gps_enu_pos_.push_back(tmp);
                ROS_INFO("global gps %d %f %f %f %f ", i, gps_pos_matrix_.at(i).x,
                                                             gps_pos_matrix_.at(i).y,
                                                             gps_pos_matrix_.at(i).z,
                                                             gps_pos_matrix_.at(i).c);

                ROS_INFO("local enu %d %f %f %f %f ", i, tmp.pose.position.x,
                                                         tmp.pose.position.y,
                                                         tmp.pose.position.z,
                                                         tf::getYaw(tmp.pose.orientation));
            }
        }
        gps_pos_enu_set_flag_ = true;
        gps_origin_counter_++;
    }
}

void TaskManager::initGlobalWaypoint(XmlRpc::XmlRpcValue &windowgps_matrix)
{
    gps_pos_matrix_.clear();//
    ROS_INFO("-- Loading gps waypoings ----- ");

    //load lat, lon, alt and yaw only
    for (size_t i = 0; i < windowgps_matrix.size(); i++)
    {
        XmlRpc::XmlRpcValue data_list(windowgps_matrix[i]);
        double temp;

        REFERENCE tmp;
        temp = data_list[0]; tmp.x = (float)temp;
        temp = data_list[1]; tmp.y = (float)temp;
        temp = data_list[2]; tmp.z = (float)temp;
        temp = data_list[3]; tmp.c = (float)temp;

        gps_pos_matrix_.push_back(tmp);
        ROS_INFO("gps waypoings %d %f %f %f %f ", i, tmp.x, tmp.y, tmp.z, tmp.c);
    }
}

void TaskManager::printWaypoints()
{
    ROS_INFO("------------current enu waypoints!!-----------");
    for(int i = 0; i < waypoints.size(); i++)
    {
       ROS_INFO("local enu %d %f %f %f %f ", i, waypoints.at(i).pose.position.x,
                                                 waypoints.at(i).pose.position.y,
                                                 waypoints.at(i).pose.position.z,
                                                 tf::getYaw(waypoints.at(i).pose.orientation));
    }
}

/**
 * @brief convert NED to ENU TaskManager::convertNED2ENU
 * @param x ned x
 * @param y ned y
 * @param z ned z
 */
void TaskManager::convertNED2ENU(float *x, float *y, float *z)
{
    float xtemp,ytemp,ztemp;
    xtemp=*x;ytemp=*y;ztemp=*z;

    *x=ytemp;
    *y=xtemp;
    *z=-ztemp;
}


// generate waypoints list from interst points' gps position
void TaskManager::convertWindowPosToWaypoint()
{
    // waypoint type: local = 1; global    = 0; default: local

    // Takeoff point
    // four points arount the corner

    waypoints.clear();

    geometry_msgs::PoseStamped tmpRef;

    // first waypoint above takeoff point
    tmpRef = current_local_pos;
    tmpRef.pose.position.z = tmpRef.pose.position.z + cruise_height_;

    subRefOrigin(&tmpRef);
    waypoints.push_back(tmpRef);

    // for all the window, add front and back waypoints
    for(int i = 0; i < gps_enu_pos_.size();i++)
    {
        // two front points and one back waypoints;
        // 1: back-right
        // 2: front-right
        // 3: front-left
        // 4: back-left
        // body in Front-Left-Up frame: FLU
        tf::Vector3 back_right_vec1(-waypoint_width_, -waypoint_width_, cruise_height_);
        tf::Vector3 front_right_vec2(waypoint_width_, -waypoint_width_, cruise_height_);
        tf::Vector3 front_left_vec3(waypoint_width_, waypoint_width_, cruise_height_);
        tf::Vector3 back_lef_vec4(-waypoint_width_, waypoint_width_, cruise_height_);

        // 1: back-right
        tf::Vector3 temp;
        temp = convertBodyTargetToENU(gps_enu_pos_.at(i).pose.position.x,
                                      gps_enu_pos_.at(i).pose.position.y,
                                      gps_enu_pos_.at(i).pose.position.z - m_ref_origin_.pose.position.z,
                                      tf::getYaw(current_local_pos.pose.orientation),
                                      back_right_vec1);
        tmpRef.pose.position.x = temp.getX();
        tmpRef.pose.position.y = temp.getY();
        tmpRef.pose.position.z = temp.getZ();

        tmpRef.pose.orientation = current_local_pos.pose.orientation;
        waypoints.push_back(tmpRef);

        // 2: front-right
        temp = convertBodyTargetToENU(gps_enu_pos_.at(i).pose.position.x,
                                      gps_enu_pos_.at(i).pose.position.y,
                                      gps_enu_pos_.at(i).pose.position.z - m_ref_origin_.pose.position.z,
                                      tf::getYaw(current_local_pos.pose.orientation),
                                      front_right_vec2);
        tmpRef.pose.position.x = temp.getX();
        tmpRef.pose.position.y = temp.getY();
        tmpRef.pose.position.z = temp.getZ();

        tmpRef.pose.orientation = current_local_pos.pose.orientation;
        waypoints.push_back(tmpRef);

        // 3: front-left
        temp = convertBodyTargetToENU(gps_enu_pos_.at(i).pose.position.x,
                                      gps_enu_pos_.at(i).pose.position.y,
                                      gps_enu_pos_.at(i).pose.position.z - m_ref_origin_.pose.position.z,
                                      tf::getYaw(current_local_pos.pose.orientation),
                                      front_left_vec3);
        tmpRef.pose.position.x = temp.getX();
        tmpRef.pose.position.y = temp.getY();
        tmpRef.pose.position.z = temp.getZ();

        tmpRef.pose.orientation = current_local_pos.pose.orientation;
        waypoints.push_back(tmpRef);

        // 4: back_lef_vec
        temp = convertBodyTargetToENU(gps_enu_pos_.at(i).pose.position.x,
                                      gps_enu_pos_.at(i).pose.position.y,
                                      gps_enu_pos_.at(i).pose.position.z - m_ref_origin_.pose.position.z,
                                      tf::getYaw(current_local_pos.pose.orientation),
                                      back_lef_vec4);
        tmpRef.pose.position.x = temp.getX();
        tmpRef.pose.position.y = temp.getY();
        tmpRef.pose.position.z = temp.getZ();

        tmpRef.pose.orientation = current_local_pos.pose.orientation;
        waypoints.push_back(tmpRef);
    }
}

tf::Vector3 TaskManager::convertBodyTargetToENU(double x, double y, double z, double c, tf::Vector3 input)
{
    tf::Transform tf_map_uav;
    tf_map_uav.setOrigin(tf::Vector3(x, y, z));

    tf::Quaternion q;
    q.setRPY(0, 0, c);
    tf_map_uav.setRotation(q);

    tf::Vector3 tf_transformed  = tf_map_uav*input;
    return tf_transformed;
}
