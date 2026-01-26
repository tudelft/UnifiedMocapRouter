// Copyright 2024 Anton Bredenbeck (Delft University of Technology)
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#ifndef ROS2_AGENT_HPP
#define ROS2_AGENT_HPP

#include "agent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class ROS2Agent : public Agent, public rclcpp::Node
{
public:
    ROS2Agent() : Node("mocap_publisher",
                         rclcpp::NodeOptions().arguments(
                                {"--ros-args",
                                "--disable-rosout-logs",
                                "--log-level", "rclcpp:=error"})),
                    _topics{"/mocap"}                 
    {
        this->name = "ros2";
    }

    void banner() override
    {
        std::cout<< R"(
##  ___  ___  ___   ___  ##
## | _ \/ _ \/ __| |_  ) ##
## |   / (_) \__ \  / /  ##
## |_|_\\___/|___/ /___| ##                     
###########################)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("publish_topics,t", boost::program_options::value<std::vector<std::string> >()->multitoken(), "ROS2 topics to publish on.");
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("publish_topics")) {
            this->_topics  = vm["publish_topics"].as<std::vector<std::string> >();
            std::cout << "ROS2 publishing topics: { ";
            for(std::string topic : this->_topics) std::cout << topic << " ";
            std::cout << "}" << std::endl;
        } else {
            std::cout << "No ROS2 Topics specified, Defaulting to: { ";
            for(std::string topic : this->_topics) std::cout << topic << " ";
            std::cout << "}" << std::endl;
        }
    }

    void pre_start() override
    {
        if(this->_topics.size() != this->streaming_ids.size())
        {
            std::cerr << "Number of topics does not match number of streaming ids. Exiting." << std::endl;
            std::raise(SIGINT);
        }

        for(unsigned int i = 0; i < this->_topics.size(); i++)
        {
            this->_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>((this->_topics.at(i) + ("/pose")).c_str(), 10));
            this->_twist_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>((this->_topics.at(i) + ("/twist")).c_str(), 10));
        } 
        this->initialized = true;
    }

    bool publish_data(int idx, pose_t& pose, twist_t& twist) override
    {
        // Update the publishing duration to more closely achieve the desired publishing frequency        
        uint64_t t = rclcpp::Clock(RCL_STEADY_TIME).now().nanoseconds();
        if (this->_last_timestamp != 0) {
            this->publish_duration += 0.1 * (1.f / this->publish_frequency - 1e-9 * (t - this->_last_timestamp));
        }
        this->_last_timestamp = t;
    
        // Init Message
        geometry_msgs::msg::PoseStamped pose_msg{};
        geometry_msgs::msg::TwistStamped twist_msg{};

        // Transform the pose timestamp to unix time
        double seconds = this->time_offset;
        uint32_t seconds_t = static_cast<int32_t>(seconds);
        uint32_t nanoseconds_t = static_cast<int32_t>((seconds - seconds_t) * 1e9);
        rclcpp::Time stamp = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - rclcpp::Duration(seconds_t, nanoseconds_t));

        // Init msg timestamp
        pose_msg.header.stamp = stamp;
        twist_msg.header.stamp = stamp;

        // Init Frame ID
        pose_msg.header.frame_id = "world";
        twist_msg.header.frame_id = "world";

        // Save in message
        pose_msg.pose.position.x = pose.x;
        pose_msg.pose.position.y = pose.y;
        pose_msg.pose.position.z = pose.z;

        pose_msg.pose.orientation.w = pose.qw;
        pose_msg.pose.orientation.x = pose.qx;
        pose_msg.pose.orientation.y = pose.qy;
        pose_msg.pose.orientation.z = pose.qz;

        twist_msg.twist.linear.x = twist.vx;
        twist_msg.twist.linear.y = twist.vy;
        twist_msg.twist.linear.z = twist.vz;

        twist_msg.twist.angular.x = twist.wx;
        twist_msg.twist.angular.y = twist.wy;
        twist_msg.twist.angular.z = twist.wz;

        // Finally publish
        this->_pose_publishers.at(idx)->publish(pose_msg);
        this->_twist_publishers.at(idx)->publish(twist_msg);

        return true;
    }

private:
    std::vector<std::string> _topics;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> _pose_publishers;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> _twist_publishers;

    uint64_t _last_timestamp;
};

#endif //ROS2_AGENT_HPP