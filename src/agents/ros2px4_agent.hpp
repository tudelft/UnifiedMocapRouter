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

#ifndef ROS2PX4_AGENT_HPP
#define ROS2PX4_AGENT_HPP

#include "agent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

class ROS2PX4Agent : public Agent, public rclcpp::Node
{
public:
    ROS2PX4Agent() : Node("mocap_publisher",
                         rclcpp::NodeOptions().arguments(
                                {"--ros-args",
                                "--disable-rosout-logs",
                                "--log-level", "rclcpp:=error"})),
                    _px4_topics{"/fmu/in/vehicle_visual_odometry"}
                  
    {
        this->name = "ros2px4";
    }

    void banner() override
    {
        std::cout<< R"(
##  ___  ___  ___   ___         _____  ___ _   ##
## | _ \/ _ \/ __| |_  )  ___  | _ \ \/ / | |  ##
## |   / (_) \__ \  / /  |___| |  _/>  <|_  _| ##
## |_|_\\___/|___/ /___|       |_| /_/\_\ |_|  ##                                                                              
#################################################)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("px4_topics,o", boost::program_options::value<std::vector<std::string> >()->multitoken(), "ROS2 topics to publish the px4 msgs on")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("px4_topics")) {
            this->_px4_topics  = vm["px4_topics"].as<std::vector<std::string> >();
            std::cout << "PX4 publishing topics: {";
            for(std::string topic : this->_px4_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        } 
        else {
            std::cout << "No PX4 Topics specified, Defaulting to: {";
            for(std::string topic : this->_px4_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        }
    }

    void pre_start() override
    {
        if(this->_px4_topics.size() != this->streaming_ids.size())
        {
            std::cerr << "Number of px4 topics does not match number of streaming ids. Exiting." << std::endl;
            std::raise(SIGINT);
        }
        for(unsigned int i = 0; i < this->_px4_topics.size(); i++)
        {
            this->_px4_publishers.push_back(this->create_publisher<px4_msgs::msg::VehicleOdometry>(this->_px4_topics.at(i), 10));
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

        px4_msgs::msg::VehicleOdometry px4_vehicle_odometry;

        // Transform the pose timestamp to unix time
        double seconds = this->time_offset;
        uint32_t seconds_t = static_cast<int32_t>(seconds);
        uint32_t nanoseconds_t = static_cast<int32_t>((seconds - seconds_t) * 1e9);
        rclcpp::Time stamp = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - rclcpp::Duration(seconds_t, nanoseconds_t));

        // Timestamp since system start (px4) in microseconds
        px4_vehicle_odometry.timestamp = stamp.nanoseconds() * 1e-3;
        px4_vehicle_odometry.timestamp_sample = px4_vehicle_odometry.timestamp;

        // Frame definition
        px4_vehicle_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        /* Then store */
        px4_vehicle_odometry.position = {pose.x, pose.y, pose.z};
        px4_vehicle_odometry.q = {pose.qw, pose.qx, pose.qy, pose.qz};
        px4_vehicle_odometry.velocity = {twist.vx, twist.vy, twist.vz};
        px4_vehicle_odometry.angular_velocity = {twist.wx, twist.wy, twist.wz};

        // Publish
        this->_px4_publishers.at(idx)->publish(px4_vehicle_odometry);

        return true;
    }

private:

    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr> _px4_publishers;

    std::vector<std::string> _px4_topics;
    std::vector<unsigned int> _px4_streaming_ids;

    uint64_t _last_timestamp;

};

#endif //ROS2PX4_AGENT_HPP