#ifndef ROS2_CLIENT_HPP
#define ROS2_CLIENT_HPP

#include "unified_mocap_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef USE_CLIENT_ROS2PX4
#include <chrono>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#endif

class Mocap2Ros2 : public UnifiedMocapClient, public rclcpp::Node
{
public:
    Mocap2Ros2() : Node("mocap_publisher",
                         rclcpp::NodeOptions().arguments(
                                {"--ros-args",
                                "--disable-rosout-logs",
                                "--log-level", "rclcpp:=error"})),
                    _topics{"/mocap"}
    {  
        std::cout<< R"(
##  ___  ___  ___   ___  ########################################################
## | _ \/ _ \/ __| |_  ) ##
## |   / (_) \__ \  / /  ##
## |_|_\\___/|___/ /___| ##
###########################
)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("publish_topics, t", boost::program_options::value<std::vector<std::string> >()->multitoken(), "ROS2 topics to publish on.")
#ifdef USE_CLIENT_ROS2PX4
            ("px4_ids, p", boost::program_options::value<std::vector<unsigned int> >()->multitoken(), "streaming ids to publish on px4 topics")
#endif
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("publish_topics")) {
            this->_topics  = vm["publish_topic"].as<std::vector<std::string> >();
            std::cout << "ROS2 publishing topics: {";
            for(std::string topic : this->_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        } else {
            std::cout << "No ROS2 Topics specified, Defaulting to: {";
            for(std::string topic : this->_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        }
#ifdef USE_CLIENT_ROS2PX4
        if(vm.count("px4_ids"))
        {
            this->_px4_streaming_ids = vm["px4_ids"].as<std::vector<unsigned int>>();
            std::cout << "PX4 streaming IDs set to";
            for(unsigned int id : this->_px4_streaming_ids) std::cout << " " << id << " ";
            std::cout << std::endl;

            if(this->_px4_streaming_ids.size() > 1)
            {
                std::cerr << "Currently not supporting multiple PX4 vehicles. Shutting down..." << std::endl;
                std::raise(SIGINT);
            }
        }
        else
        {   
            // By default we take the first streaming id
            this->_px4_streaming_ids.push_back(this->get_streaming_ids().at(0));

            std::cout << "PX4 Streaming IDs not set, defaulting to";
            for(unsigned int id : this->_px4_streaming_ids) std::cout << " " << id << " ";
            std::cout << std::endl;
        }
#endif
    }

    void pre_start() override
    {
        if(this->_topics.size() != this->get_streaming_ids().size())
        {
            std::cerr << "Number of topics does not match number of streaming ids. Exiting." << std::endl;
            std::raise(SIGINT);
        }

        for(unsigned int i = 0; i < this->_topics.size(); i++)
        {
            this->_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>((this->_topics.at(i) + ("/pose")).c_str(), 10));
            this->_twist_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>((this->_topics.at(i) + ("/twist")).c_str(), 10));
        } 

#ifdef USE_CLIENT_ROS2PX4
        this->_px4_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
#endif
    }

    void publish_data() override
    {
        // Copy the streaming ids
        std::vector<unsigned int> streaming_ids = this->get_streaming_ids();

        for(unsigned int i = 0; i < this->_pose_publishers.size(); i++)
        {   
            // Only publish messages if the current value is valid and has not been published before
            if(this->isUnpublishedRB(i))
            {
                // Get the current pose and twist
                pose_t pose = this->getPoseRB(i);
                pose_der_t twist = this->getPoseDerRB(i);

                // Init Message
                geometry_msgs::msg::PoseStamped pose_msg{};
                geometry_msgs::msg::TwistStamped twist_msg{};

                // Transform the pose timestamp to unix time
                double seconds = this->seconds_since_mocap_ts(pose.timeUs);
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

                twist_msg.twist.linear.x = twist.x;
                twist_msg.twist.linear.y = twist.y;
                twist_msg.twist.linear.z = twist.z;
                
                twist_msg.twist.angular.x = twist.wx;
                twist_msg.twist.angular.y = twist.wy;
                twist_msg.twist.angular.z = twist.wz;

                #ifdef USE_CLIENT_ROS2PX4
                /* If the current object is also in the list of ids
                 * that should be published on a px4 topic */
                if(std::find(this->_px4_streaming_ids.begin(),
                             this->_px4_streaming_ids.end(),
                             streaming_ids.at(i)) != this->_px4_streaming_ids.end())
                {
                    px4_msgs::msg::VehicleOdometry px4_vehicle_odometry;

                    // Timestamp since system start (px4) in microseconds
                    px4_vehicle_odometry.timestamp = stamp.nanoseconds() * 1e-3;
                    px4_vehicle_odometry.timestamp_sample = px4_vehicle_odometry.timestamp;

                    // Frame definition
                    px4_vehicle_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

                    // Pose and Twist
                    /* First transform to NED */
                    pose = this->toNED(pose);
                    twist = this->toNED(twist);

                    /* Then store */
                    px4_vehicle_odometry.position = {pose.x, pose.y, pose.z};
                    px4_vehicle_odometry.q = {pose.qw, pose.qx, pose.qy, pose.qz};
                    px4_vehicle_odometry.velocity = {twist.x, twist.y, twist.z};
                    px4_vehicle_odometry.angular_velocity = {twist.wx, twist.wy, twist.wz};

                    // Publish
                    this->_px4_publisher->publish(px4_vehicle_odometry);
                }
                #endif

                // Finally publish
                this->_pose_publishers.at(i)->publish(pose_msg);
                this->_twist_publishers.at(i)->publish(twist_msg);
            }
        }


      
    }

private:
    std::vector<std::string> _topics;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> _pose_publishers;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> _twist_publishers;

#ifdef USE_CLIENT_ROS2PX4
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _px4_publisher;

    std::vector<unsigned int> _px4_streaming_ids;


    pose_t toNED(const pose_t pose) const
    {
        pose_t result = pose;

        switch(this->getCO())
        {
            case CoordinateSystem::ENU:
                result.x = pose.y;
                result.y = pose.x;
                result.z = -pose.z;

                result.qw = pose.qw;
                result.qx = pose.qy;
                result.qy = pose.qx;
                result.qz = -pose.qz;
                break;
            case CoordinateSystem::NED:
                // We do nothing because this is what we want to have
                break;
            default:
                break;
        }

        return result;
    }

    pose_der_t toNED(const pose_der_t pose_der) const
    {
        pose_der_t result = pose_der;

        switch(this->getCO())
        {
            case CoordinateSystem::ENU:
                result.x = pose_der.y;
                result.y = pose_der.x;
                result.z = -pose_der.z;

                result.wx = pose_der.wy;
                result.wy = pose_der.wx;
                result.wz = -pose_der.wz;
                break;
            case CoordinateSystem::NED:
                // We do nothing because this is what we want to have
                break;
            default:
                break;
        }

        return result;
    }

#endif

};

#endif //ROS2_CLIENT_HPP