#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/DisplayTrajectory.h"

#define VIS_TOPIC "joint_states_visualizer"
#define SPEED_FACTOR .5

class Trajectory_to_joint_states
{
    ros::Subscriber display_trajectory_sub;
    ros::Publisher joint_state_pub;
    trajectory_msgs::JointTrajectory current_trajectory;
    ros::Time display_start_time;

public:
    Trajectory_to_joint_states(ros::NodeHandle* n);

    void display_trajectory_callback(const moveit_msgs::DisplayTrajectoryConstPtr& msg);
    void spin();
};