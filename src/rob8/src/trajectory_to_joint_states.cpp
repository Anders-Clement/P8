#include "trajectory_to_joint_states.h"


void Trajectory_to_joint_states::display_trajectory_callback(const moveit_msgs::DisplayTrajectoryConstPtr& msg)
{
    current_trajectory.joint_names = msg->trajectory[0].joint_trajectory.joint_names;
    current_trajectory.header = msg->trajectory[0].joint_trajectory.header;
    current_trajectory.points.clear();

    // append all points from all trajectories:
    for(auto trajectory : msg->trajectory)
    {
        // trajectory.joint_trajectory.
        for(auto point : trajectory.joint_trajectory.points)
            current_trajectory.points.push_back(point);
    }

    display_start_time = ros::Time::now();
    float duration = current_trajectory.points.back().time_from_start.toSec();
    ROS_INFO("path duration: %f, average points per sec: %f",
        duration,
        (float)current_trajectory.points.size()/duration);
}

Trajectory_to_joint_states::Trajectory_to_joint_states(ros::NodeHandle* nh)
{
    display_trajectory_sub = nh->subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, &Trajectory_to_joint_states::display_trajectory_callback, this);
    joint_state_pub = nh->advertise<sensor_msgs::JointState>(VIS_TOPIC, 5, true);
}

void Trajectory_to_joint_states::spin()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if(current_trajectory.points.empty())
            continue;

        // find point we are supposed to be at now
        trajectory_msgs::JointTrajectoryPoint* current_point;
        auto now = ros::Time::now();
        auto display_run_time = now - display_start_time;

        for(int i = 0; i < current_trajectory.points.size(); i++)
        {
            if(current_trajectory.points[i].time_from_start.toNSec() * SPEED_FACTOR < display_run_time.toNSec())
                current_point = &current_trajectory.points[i];
            else
                break;
        }

        if(current_point == nullptr || *current_point == current_trajectory.points[current_trajectory.points.size()-1])
        {
            display_start_time = ros::Time::now();
            // ROS_INFO("done, looping around");
            continue;
        }

        // ROS_INFO("now: %d", (int)now.toSec());
        // ROS_INFO("display_start: %d", (int)display_start_time.toSec());
        // ROS_INFO("time_from_start: %d", (int)current_point->time_from_start.toSec());


        // convert trajectory point to jointstate msg
        sensor_msgs::JointState msg;
        msg.header = current_trajectory.header;
        msg.position = current_point->positions;
        msg.name = current_trajectory.joint_names;

        joint_state_pub.publish(msg);
        for(int i = 0; i < current_point->positions.size(); i++)
        {
            current_point->positions[i] += current_point->velocities[i];
            current_point->velocities[i] += current_point->accelerations[i];
        }
    }
}





int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_to_joint_states");
    auto nh = ros::NodeHandle();
    auto traj_to_joint_states = Trajectory_to_joint_states(&nh);
    traj_to_joint_states.spin();
    return 0;
}