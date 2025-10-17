// Copyright 2024 Till Blaha, Anton Bredenbeck (Delft University of Technology)
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

#include <boost/filesystem.hpp>

#include "unified_mocap_router.hpp"
#include "mocap.hpp"
#include "agent.hpp"


#ifdef USE_MOCAP_TEST
    #include "test_mocap.hpp"
#endif

#ifdef USE_MOCAP_OPTITRACK
    #include "optitrack_mocap.hpp"
#endif

#ifdef USE_MOCAP_QUALISYS
    #include "qualisys_mocap.hpp"
#endif



#ifdef USE_AGENT_CONSOLE
    #include "console_agent.hpp"
#endif

#ifdef USE_AGENT_IVY
    #include "ivy_agent.hpp"
#endif

#ifdef USE_AGENT_UDP
    #include "udp_agent.hpp"
#endif

#ifdef USE_AGENT_MAVLINK
    #include "mavlink_agent.hpp"
#endif

#ifdef USE_AGENT_LOG
    #include "log_agent.hpp"
#endif

#ifdef USE_AGENT_ROS2
    #include "ros2_agent.hpp"
    #include "rclcpp/rclcpp.hpp"
#endif

#ifdef USE_AGENT_ROS2PX4
    #include "ros2px4_agent.hpp"
    #include "rclcpp/rclcpp.hpp"
#endif

namespace{
    std::function<void(int)> shutdown_handler;
    void signal_handler(int signal) { shutdown_handler(signal); }
}

static void print_usage(char const *prog_name) {

    // TODO: highlight that a general --help is not avaialble,
    //       you have to pass MOCAP and AGENT to get specific help
    printf("\nUsage: %s MOCAP AGENT [-h|--help] [options]\n\n", prog_name);
    printf("NOTE: A general --help is not available. Specify MOCAP and AGENT, followed by -h/--help!\n\n");
    printf("Available MOCAPs: ");
#ifdef USE_MOCAP_TEST
    printf("test ");
#endif
#ifdef USE_MOCAP_OPTITRACK
    printf("optitrack ");
#endif
#ifdef USE_MOCAP_QUALISYS
    printf("qualisys ");
#endif
    printf("\n");

    printf("Available AGENTs: ");
#ifdef USE_AGENT_CONSOLE
    printf("console ");
#endif
#ifdef USE_AGENT_LOG
    printf("log ");
#endif
#ifdef USE_AGENT_IVY
    printf("ivy ");
#endif
#ifdef USE_AGENT_UDP
    printf("udp ");
#endif
#ifdef USE_AGENT_MAVLINK
    printf("mavlink ");
#endif
#ifdef USE_AGENT_ROS2
    printf("ros2 ");
#endif
#ifdef USE_AGENT_ROS2PX4
    printf("ros2px4 ");
#endif
    printf("\n\n");
}

int main(int argc, char const *argv[])
{
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    Mocap* mocap = nullptr;
#ifdef USE_MOCAP_TEST
    if (strcasecmp(argv[1], "test") == 0) {
        mocap = new TestMocap();
    } else
#endif
#ifdef USE_MOCAP_OPTITRACK
    if (strcasecmp(argv[1], "optitrack") == 0) {
        mocap = new OptiTrackMocap();
    } else
#endif
#ifdef USE_MOCAP_QUALISYS
    if (strcasecmp(argv[1], "qualisys") == 0) {
        mocap = new QualisysMocap();
    } else
#endif
    {
        std::cout << "Support for Mocap '" << argv[1] << "' has not been compiled into the program. exiting." << std::endl;
        print_usage(argv[0]);
        return 1;
    }


    Agent* agent = nullptr;
#ifdef USE_AGENT_CONSOLE
    if (strcasecmp(argv[2], "console") == 0) {
        agent = new ConsoleAgent();
    } else
#endif
#ifdef USE_AGENT_IVY
    if (strcasecmp(argv[2], "ivy") == 0) {
        agent = new IvyAgent();
    } else 
#endif
#ifdef USE_AGENT_UDP
    if (strcasecmp(argv[2], "udp") == 0) {
        agent = new UdpAgent();
    } else 
#endif
#ifdef USE_AGENT_MAVLINK
    if (strcasecmp(argv[2], "mavlink") == 0) {
        agent = new MavlinkAgent();
    } else 
#endif
#ifdef USE_AGENT_LOG
    if (strcasecmp(argv[2], "log") == 0) {
        agent = new LogAgent();
    } else 
#endif
#ifdef USE_AGENT_ROS2   
    if (strcasecmp(argv[2], "ros2") == 0) {
        // Init ROS2
        rclcpp::init(argc-2, argv+2);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init agent
        agent = new ROS2Agent();
    } else
#endif
#ifdef USE_AGENT_ROS2PX4
    if (strcasecmp(argv[2], "ros2px4") == 0) {
        // Init ROS2
        rclcpp::init(argc-2, argv+2);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init agent
        agent = new ROS2PX4Agent();
    } else
#endif
    {
        std::cout << "Support for agent '" << argv[2] << "' was not compiled into the program. exiting." << std::endl;
        print_usage(argv[0]);
        return 1;
    }


    // enroll shutdown handler
    boost::filesystem::path p(argv[0]);
    shutdown_handler = [p, argv](int signum)
    { 
        std::cout << "Shutting down... Done. " << std::endl;
#if defined(USE_AGENT_ROS2) || defined(USE_AGENT_ROS2PX4)  
        if (strcasecmp(argv[2], "ros2") == 0 || strcasecmp(argv[2], "ros2px4") == 0 ) {
            rclcpp::shutdown();
        }
#endif
         exit(0);
    };

	signal(SIGINT, signal_handler);


    // construct and run mocap-router
    UnifiedMocapRouter router = UnifiedMocapRouter(mocap, agent);
    router.start(argc-2, argv+2); // pointer arithmetic magic

    return 0;
}
