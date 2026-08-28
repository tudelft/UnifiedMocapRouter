// Copyright 2024/25 Evangelios Ntoures, Till Blaha (Delft University of Technology)
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

#ifndef MAVLINK_AGENT_HPP
#define MAVLINK_AGENT_HPP

#include "agent.hpp"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/time.h>

#include <mavlink/common/mavlink.h>


enum MAVLinkAp { PX4=0, ARDUCOPTER, ARDUPLANE };
std::ostream& operator<<(std::ostream& lhs, MAVLinkAp e) {
    switch(e) {
    case PX4: lhs << "px4"; break;
    case ARDUCOPTER: lhs << "arducopter"; break;
    case ARDUPLANE: lhs << "arduplane"; break;
    }
    return lhs;
} 

class MavlinkAgent : public Agent
{
public:
    MavlinkAgent()
    {
        // do as little setup here as possible. Use pre_start instead
        this->name = "mavlink";
    }

    ~MavlinkAgent()
    {
        close(this->socket_fd);
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##  __  __          _ _      _    ##
## |  \/  |__ ___ _| (_)_ _ | |__ ##
## | |\/| / _` \ V / | | ' \| / / ##
## |_|  |_\__,_|\_/|_|_|_||_|_\_\ ##
####################################)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("client_ip,i", boost::program_options::value<std::string>(), "IP to stream the MAVLink UDP data to. Default 127.0.0.1")
            ("port,p", boost::program_options::value<unsigned short int>(), "Port. Default 14551 for UDP mode and 5760 for TCP mode (not recommended for vehicle uplink).")
            ("tcp", boost::program_options::bool_switch()->default_value(false), "Use TCP instead of UDP. Default is disabled/UDP.")
            // TODO: This is actually a uint8_t, but that seems to be interpreted as an ASCII character -> gives the wrong number (even unsigned char?)
            ("system_ids", boost::program_options::value<std::vector<unsigned short int>>()->multitoken(), "MAVLink destination system id(s) - should be in the same order as the streaming IDs. Default 42.")
            ("autopilot,a", boost::program_options::value<std::string>(), "One of [arducopter, arduplane, px4]")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        // Do we run on TCP or UDP?
        // Check if we want to use TCP or UDP
        if (vm.count("tcp") && vm["tcp"].as<bool>()) {
            this->use_tcp = true;
        } else {
            this->use_tcp = false;
        }

        if (vm.count("client_ip")) {
            std::string val = vm["client_ip"].as<std::string>();
            this->_client_ip = val;
        } else {
            this->_client_ip = "127.0.0.1";
        }

        if (vm.count("port")) {
            unsigned short int val = vm["port"].as<unsigned short int>();
            this->_port = val;
        } else {
            this->_port = this->use_tcp ? 5760 : 14551; // TCP/UDP defaults differ
        }

        // Log a single message with the connection details
        std::cout << "MAVLink Agent will connect to " << this->_client_ip << ":" << this->_port 
                  << " using " << (this->use_tcp ? "TCP" : "UDP") << std::endl;
        
        // System IDs, should match the number of streaming IDs
        if (vm.count("system_ids")) {
            this->_mav_system_ids = vm["system_ids"].as<std::vector<unsigned short int>>();
        } else {
            std::cout << "No MAVLink system ids given, using default of 42" << std::endl;
            this->_mav_system_ids = {42};
        }
        
        if (this->_mav_system_ids.size() != this->streaming_ids.size()) {
            std::cout << "Number of MAVLink system IDs must match the number of streaming IDs. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        else {
            // Log the streaming IDs and their corresponding MAVLink system IDs
            std::cout << "MAVLink system IDs mappings: " << std::endl;
            for (size_t i = 0; i < this->streaming_ids.size(); ++i) {
                std::cout << " - Streaming ID " << this->streaming_ids[i] << " -> MAVLink System ID " << this->_mav_system_ids[i] << std::endl;
            }
        }

        if (vm.count("autopilot")) {
            std::string ap_name = vm["autopilot"].as<std::string>();
            boost::algorithm::to_lower(ap_name);

            if (ap_name.compare("px4") == 0) { this->ap = MAVLinkAp::PX4; }
            else if (ap_name.compare("arducopter") == 0) { this->ap = MAVLinkAp::ARDUCOPTER; }
            else if (ap_name.compare("arduplane") == 0) { 
                this->ap = MAVLinkAp::ARDUPLANE; 
                std::cout << "WARNING: Autopilot type" << ap_name << " not yet supported. " << std::endl;
            } else {
                std::cout << "Autopilot type " << ap_name << " not definied. Exiting" << std::endl;
                std::raise(SIGINT);
            }
            std::cout << "Autopilot type set to " << this->ap << std::endl;
        } else {
            std::cout << "Autopilot type not set, exiting" << std::endl;
            std::raise(SIGINT);
        }
    }

    void printSocketAddress(const struct sockaddr_in *addr) {
        printf("IP Address: %s\n", inet_ntoa(addr->sin_addr));
        printf("Port: %d\n", ntohs(addr->sin_port));
    }

    void pre_start() override
    {
        // Parse the desired IP address and port - same for TCP and UDP
        struct sockaddr_in addr = {};
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        inet_pton(AF_INET, this->_client_ip.c_str(), &(addr.sin_addr));
        addr.sin_port = htons(this->_port);

        // Create the TCP/UDP socket
        socket_fd = socket(PF_INET, this->use_tcp ? SOCK_STREAM : SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            printf("socket error: %s\n", strerror(errno));
            std::raise(SIGINT);
        }
        
        // Perform TCP connection if requested
        // NOTE: No need to send a heartbeat for TCP & don't care about the port we use on this computer; consider initialized upon connection
        if (this->use_tcp) {
            int status = connect(socket_fd, (sockaddr*) &addr, sizeof(addr));
            if(status < 0)
            {
                std::cout << "Error connecting to socket!" << std::endl;
            }
            this->initialized = true;
            return;
        }
        else {
            // Local client IP -> assume we want to bind to this address
            if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
                printf("bind error: %s\n", strerror(errno));
                // return -2;
            }

            // We set a timeout at 100ms to prevent being stuck in recvfrom for too
            // long and missing our chance to send some stuff.
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
                printf("setsockopt error: %s\n", strerror(errno));
                // return -3;
            }

            // we need to wait for autopilot to be online before sending, otherwise
            // there are errors (at least with mavlink-routerd)
            std::cout << "Waiting for heartbeat..." << std::endl;

            while (!this->_heartbeat_received) {
                receive_some();
            }

            // print sender's address
            // printSocketAddress(&src_addr);

            this->initialized = true;
        }
    }

    bool publish_data(int idx, pose_t& pose, twist_t& twist) override
    {
        if (idx >= this->streaming_ids.size()) {
            // We have no mapping to streaming ID (or MAVLINK system ID for that matter) - ignore
            return false;
        }

        switch (this->ap) {
            case MAVLinkAp::PX4:
                send_vision_position_estimate(pose, this->_mav_system_ids[idx]);
                break;
            case MAVLinkAp::ARDUCOPTER:
                send_att_pos_mocap(pose, this->_mav_system_ids[idx]);
                break;
            case MAVLinkAp::ARDUPLANE:
                send_gps_input(pose, this->_mav_system_ids[idx]);
                break;
        }

        return true;
    }

    void quaternion_to_euler(const pose_t pose, float* euler) {
        // 3-2-1 rotation sequence (yaw then pitch then roll)
        // NED coordinates assumed

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (pose.qw * pose.qx + pose.qy * pose.qz);
        double cosr_cosp = 1 - 2 * (pose.qx * pose.qx + pose.qy * pose.qy);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // roll
 
        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (pose.qw * pose.qy - pose.qx * pose.qz));
        double cosp = std::sqrt(1 - 2 * (pose.qw * pose.qy - pose.qx * pose.qz));
        euler[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2; // pitch
 
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (pose.qw * pose.qz + pose.qx * pose.qy);
        double cosy_cosp = 1 - 2 * (pose.qy * pose.qy + pose.qz * pose.qz);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // yaw
    }

    void send_vision_position_estimate(pose_t& pose, uint8_t mav_system_id)
    {
        if (!src_addr_set) {
            return;
        }

        mavlink_message_t message;

        struct timeval tv;
        uint64_t timestamp_us;
        gettimeofday(&tv, NULL);
        timestamp_us = (long long)tv.tv_sec * 1000000LL + (long long)tv.tv_usec;

        float euler[3];
        quaternion_to_euler(pose, euler);

        float cov[21];
        cov[0] = NAN;

        //printf("%ld,%f,%f,%f\n", timestamp_us, pose.x, pose.y, pose.z);

        mavlink_msg_vision_position_estimate_pack(
                mav_system_id,
                MAV_COMP_ID_PERIPHERAL,
                &message,
                timestamp_us,
                pose.x,
                pose.y,
                pose.z,
                euler[0], // roll
                euler[1], // pitch
                euler[2], // yaw
                cov,
                0);

        send_mavlink_msg(message);
    }

    void send_mavlink_msg(mavlink_message_t &message)
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const int len = mavlink_msg_to_send_buffer(buf, &message);

        int ret = 0;
        if (this->use_tcp) {
            ret = send(socket_fd, buf, len, 0);
        }
        else {
            ret = sendto(socket_fd, buf, len, 0, (struct sockaddr*)(&src_addr), src_addr_len);
        }

        if (ret != len)
        {
            printf("sendto error: %s\n", strerror(errno));
        }
    }

    void send_att_pos_mocap(pose_t& pose, uint8_t mav_system_id)
    {
        if (!src_addr_set) {
            return;
        }

        mavlink_message_t message;

        struct timeval tv;
        uint64_t timestamp_us;
        gettimeofday(&tv, NULL);
        timestamp_us = (long long)tv.tv_sec * 1000000LL + (long long)tv.tv_usec;

        float q[4] = {pose.qw, pose.qx, pose.qy, pose.qz};

        float cov[21];
        cov[0] = NAN;

        //printf("%ld,%f,%f,%f\n", timestamp_us, pose.x, pose.y, pose.z);

        mavlink_msg_att_pos_mocap_pack(
                mav_system_id,
                MAV_COMP_ID_PERIPHERAL,
                &message,
                timestamp_us,
                q,
                pose.x,
                pose.y,
                pose.z,
                cov);

        
        send_mavlink_msg(message);
    }

    void send_gps_input(pose_t& pose, uint8_t mav_system_id)
    {
        if (!src_addr_set) {
            return;
        }

        mavlink_message_t message;

        struct timeval tv;
        uint64_t timestamp_us;
        gettimeofday(&tv, NULL);
        timestamp_us = (long long)tv.tv_sec * 1000000LL + (long long)tv.tv_usec;

        mavlink_msg_gps_input_pack(
            mav_system_id,
            MAV_COMP_ID_PERIPHERAL,
            &message,
            timestamp_us,
            0,
            254,
            0, // ms start of week
            1, // week no   
            3, // 3d fix
            386300000, // lat
            242000000, // lon
            0, // alt
            UINT16_MAX,
            UINT16_MAX,
            0,0,0,0,0,0,
            15,
            0); //yaw

        
        send_mavlink_msg(message);
    }

    void receive_some()
    {
        // We just receive one UDP datagram and then return again.
        char buffer[2048]; // enough for MTU 1500 bytes

        const int ret = recvfrom(
                socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(&src_addr), &src_addr_len);

        if (ret < 0) {
            //printf("recvfrom error: %s\n", strerror(errno));
            return;
        } else if (ret == 0) {
            // peer has done an orderly shutdown
            return;
        } 

        src_addr_set = true;

        mavlink_message_t message;
        mavlink_status_t status;
        for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

                // printf(
                //     "Received message %d from %d/%d\n",
                //     message.msgid, message.sysid, message.compid);

                switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    handle_heartbeat(&message);
                    break;
                }
            }
        }
    }

    void handle_heartbeat(const mavlink_message_t* message)
    {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(message, &heartbeat);

        printf("Got heartbeat from ");
        switch (heartbeat.autopilot) {
            case MAV_AUTOPILOT_GENERIC:
                printf("generic");
                break;
            case MAV_AUTOPILOT_ARDUPILOTMEGA:
                printf("ArduPilot");
                break;
            case MAV_AUTOPILOT_PX4:
                printf("PX4");
                break;
            default:
                printf("other (MAV_AUTOPILOT=%d)", heartbeat.autopilot);
                break;
        }
        printf(" autopilot\n");
        // TODO: Parse out system IDs and ensure all system IDs have received a heartbeat before clearing this wait?
        this->_heartbeat_received = true;
    }

private:
    bool use_tcp;
    unsigned short int _port;
    std::string _client_ip;
    std::vector<unsigned short int> _mav_system_ids;

    int socket_fd;
    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = true;
    MAVLinkAp ap;
    bool _heartbeat_received = false;
};

#endif // MAVLINK_AGENT_HPP