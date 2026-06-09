/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <pairs_msgs/MpcTrackerDiagnostics.h>
#include <pairs_msgs/ReferenceStampedSrv.h>
#include <pairs_msgs/TrackerCommand.h>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/profiler.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/attitude_converter.h>

#include <pairs_lib/subscribe_handler.h>
#include <pairs_lib/transformer.h>

//}

namespace pairs_rviz_plugins
{

namespace rviz_interface
{

/* class NavGoal //{ */

class NavGoal : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

private:
  pairs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sh_rviz_goal_;

  void callbackRvizNavGoal(const geometry_msgs::PoseStamped::ConstPtr wrp);

  pairs_lib::SubscribeHandler<pairs_msgs::TrackerCommand> sh_tracker_cmd_;

  void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);

  bool       got_odom_uav_ = false;
  std::mutex mutex_odom_uav_;

  pairs_lib::Transformer transformer_;

private:
  pairs_lib::Profiler* profiler_;
  bool               _profiler_enabled_ = false;

  ros::ServiceClient srv_client_reference_;
};

//}

/* onInit() //{ */

void NavGoal::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  pairs_lib::ParamLoader param_loader(nh_, "NavGoal");

  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  param_loader.loadParam("uav_name", _uav_name_);

  transformer_ = pairs_lib::Transformer("NavGoal");
  transformer_.setLookupTimeout(ros::Duration(1.0));
  transformer_.setDefaultFrame(_uav_name_);

  pairs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "NavGoal";
  shopts.no_message_timeout = pairs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 5;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();


  // | ----------------------- subscribers ---------------------- |
  sh_rviz_goal_   = pairs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "rviz_nav_goal_in", &NavGoal::callbackRvizNavGoal, this);
  sh_tracker_cmd_ = pairs_lib::SubscribeHandler<pairs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(1.0), &NavGoal::timeoutTrackerCmd, this);

  // | --------------- initialize service clients --------------- |
  srv_client_reference_ = nh_.serviceClient<pairs_msgs::ReferenceStampedSrv>("reference_service_out");

  // | ----------------------- finish init ---------------------- |
  is_initialized_ = true;

  ROS_INFO_ONCE("[NavGoal]: initialized");

  ROS_INFO("[RvizPoseEstimate]: Waiting for user input in rviz...");
}
//}

// | ---------------------- msg callbacks --------------------- |

/* callbackRvizNavGoal() //{ */

void NavGoal::callbackRvizNavGoal(const geometry_msgs::PoseStamped::ConstPtr msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  if (!sh_tracker_cmd_.hasMsg()) {
    ROS_WARN("[NavGoal]: Haven't received tracker command yet, skipping goal.");
    return;
  }

  geometry_msgs::PoseStamped goal = *msg;

  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  // transform UAV odometry to reference frame of the goal
  auto tf = transformer_.getTransform(tracker_cmd->header.frame_id, goal.header.frame_id, tracker_cmd->header.stamp);

  if (tf) {

    geometry_msgs::PoseStamped pose_uav;
    pose_uav.header           = tracker_cmd->header;
    pose_uav.pose.position    = tracker_cmd->position;
    pose_uav.pose.orientation = tracker_cmd->orientation;

    auto res = transformer_.transform(pose_uav, tf.value());

    if (res) {
      // set z-coordinate of the goal to be the same as in cmd odom
      goal.pose.position.z = res.value().pose.position.z;
      ROS_INFO("[NavGoal]: Setting z = %.3f m to the goal, frame_id of goal: %s", goal.pose.position.z, goal.header.frame_id.c_str());
    } else {
      ROS_WARN("[NavGoal]: Unable to transform tracker command from %s to %s at time %.6f.", tracker_cmd->header.frame_id.c_str(), goal.header.frame_id.c_str(),
               tracker_cmd->header.stamp.toSec());
      return;
    }

  } else {
    ROS_WARN("[NavGoal]: Unable to find transform from %s to %s at time %.6f.", tracker_cmd->header.frame_id.c_str(), goal.header.frame_id.c_str(),
             tracker_cmd->header.stamp.toSec());
    return;
  }

  // publish the goal to the service
  // Create new waypoint msg
  pairs_msgs::ReferenceStampedSrv new_waypoint;
  new_waypoint.request.header.frame_id    = goal.header.frame_id;
  new_waypoint.request.header.stamp       = ros::Time::now();
  new_waypoint.request.reference.position = goal.pose.position;

  try {
    new_waypoint.request.reference.heading = pairs_lib::AttitudeConverter(goal.pose.orientation).getHeading();
  }
  catch (pairs_lib::AttitudeConverter::GetHeadingException& e) {
    /* new_waypoint.request.reference.heading = 0; */
    ROS_ERROR("[NavGoal]: Unable to calculate heading from quaternion: [%.3f %.3f %.3f %.3f]", goal.pose.orientation.x, goal.pose.orientation.y,
              goal.pose.orientation.z, goal.pose.orientation.w);
    return;
  }

  ROS_INFO("[NavGoal]: Calling reference service with point [%.3f %.3f %.3f], heading: %.3f", new_waypoint.request.reference.position.x,
           new_waypoint.request.reference.position.y, new_waypoint.request.reference.position.z, new_waypoint.request.reference.heading);
  bool success = srv_client_reference_.call(new_waypoint);
  ROS_INFO("[NavGoal]: Reference service response: %s", new_waypoint.response.message.c_str());

  if (!success || !new_waypoint.response.success) {
    ROS_ERROR("[NavGoal]: Could not set reference.");
    return;
  }
}

//}

/* timeoutTrackerCmd() //{ */

void NavGoal::timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg) {

  ROS_WARN_THROTTLE(1.0, "[NavGoal]: not receiving '%s' for %.3f s", topic.c_str(), (ros::Time::now() - last_msg).toSec());
}

//}

}  // namespace rviz_interface

}  // namespace pairs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_rviz_plugins::rviz_interface::NavGoal, nodelet::Nodelet);
