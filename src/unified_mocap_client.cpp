// Copyright 2024 Anton Bredenbeck, Till Blaha (Delft University of Technology)
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

#include "unified_mocap_client.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <boost/algorithm/string.hpp>
#include <functional>

#include <unistd.h>
#include <termios.h>
#include <inttypes.h>

#include "mocap.hpp"
#include "agent.hpp"

namespace po = boost::program_options;

UnifiedMocapClient::UnifiedMocapClient(const UnifiedMocapClient &other)
{
    (void) other;
    std::cerr << "Copy constructor for UnifiedMocapClient not supported. Exiting." << std::endl;
    std::raise(SIGINT);
}

UnifiedMocapClient::UnifiedMocapClient(Mocap* mocap, Agent* agent) :
    printMessages{false}, desc{"Allowed options"}, vm{}, publish_div{0}, publish_frequency{0.f}
{
    // TODO: use builtin forward prediction with the latency estimates plus a 
    // user-defined interval (on the order of 10ms)?
    this->mocap = mocap;
    this->agent = agent;
    this->mocap->enroll_agent(this->agent);

    this->banner();
    this->mocap->banner();
    this->agent->banner();
}

void UnifiedMocapClient::keystroke_loop()
{
    // wait for keystrokes
    std::cout << std::endl << "Listening to messages! Press q to quit, Press t to toggle message printing" << std::endl;

	while ( const int c = getch() )
    {
        switch(c)
        {
            case 'q':
                std::raise(SIGINT);
                break;
            case 't':
                this->agent->togglePrintMessages();
                break;
        }
    }
}

void UnifiedMocapClient::start(int argc, const char *argv[])
{
    this->add_base_po();
    this->mocap->add_extra_po(this->desc);
    this->agent->add_extra_po(this->desc);

    this->parse_base_po(argc, argv);
    this->mocap->parse_extra_po(this->vm);
    this->agent->parse_extra_po(this->vm);

    int ret = this->mocap->connect();
    if (ret != 0) {
        std::cout << "Error connecting to mocap server. exiting" << std::endl;
        std::raise(SIGINT); 
    }

    this->agent->pre_start();
    this->keyThread = std::thread(&UnifiedMocapClient::keystroke_loop, this);
    this->agent->post_start();

    // some blocking code
    this->keyThread.join();
}

void UnifiedMocapClient::add_base_po()
{
    this->desc.add_options()
        ("help,h", "produce help message")
        ("publish_divisor,d", po::value<unsigned int>(), "Publish every <publish_divisor> sample of the incoming MoCap data")
        ("publish_frequency,f", po::value<float>(), "Publish at least at this frequency, if there are new samples.")
        ("coordinate_system,c", po::value<std::string>(), "coordinate system convention to use [ned, enu]")
        ("coordinate_north,r", po::value<std::string>(), "where north should be relative to the observer. Either non-zero number describing right-hand rotation of north axis from the far side, or one of [right, far_side, left, near_side].")
        ("streaming_ids,s", po::value<std::vector<unsigned int>>()->multitoken(), "streaming ids to track")
        ("craft_noses,n", po::value<std::vector<std::string>>()->multitoken(), "direction of aircraft noses when creating the rigid body in the mocap software. space-separated list of [right, far_side, left, near_side]")
    ;
}

void UnifiedMocapClient::parse_base_po(int argc, char const *argv[])
{
    po::store(po::parse_command_line(argc, argv, this->desc), this->vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(0);
    }

    CoordinateSystem co;
    if(vm.count("coordinate_system"))
    {
        std::string co_name = vm["coordinate_system"].as<std::string>();
        boost::algorithm::to_lower(co_name);

        //if(co_name.compare("unchanged") == 0) { this->co = CoordinateSystem::UNCHANGED; }
        //else
        if(co_name.compare("ned") == 0) { co = CoordinateSystem::NED; }
        else if (co_name.compare("enu") == 0) { co = CoordinateSystem::ENU; }
        else {
            std::cout << "Coordinate system " << co_name << " not definied. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::cout << "Coordinate system set to " << co << std::endl;
    }
    else
    {
        std::cout << "Coordinate System not set, exiting" << std::endl;
                  //<< co << std::endl;
        std::raise(SIGINT);
    }

    this->agent->set_csys(co);

    float true_north_rad = 0.f;
    if(vm.count("coordinate_north"))
    {
        //if (this->co == CoordinateSystem::UNCHANGED) {
        //    std::cout << "Can only specify --coordinate_north/-r when coordinate system is not UNCHANGED. Exiting" << std::endl;
        //    std::raise(SIGINT);
        //}
        std::string co_north = vm["coordinate_north"].as<std::string>();
        boost::algorithm::to_lower(co_north);

        if(co_north.compare("right") == 0)
        {
            true_north_rad = -M_PI/2.0f;
        }
        else if(co_north.compare("far_side") == 0)
        {
            true_north_rad = 0.f;
        }
        else if (co_north.compare("left") == 0)
        {
            true_north_rad = +M_PI/2.0f;
        }
        else if (co_north.compare("near_side") == 0)
        {
            true_north_rad = +M_PI;
        }
        else
        {
            true_north_rad = M_PI/180.f * std::atof(co_north.c_str());
            if (true_north_rad == 0.0) {
                std::cout << "Coordinate system argument " << co_north << " is neither [near_side, far_side, right, left], nor float (for 0.0 use far_side). Exiting" << std::endl;
                std::raise(SIGINT);
            }

        }

        std::cout << "Coordinate system north set to " << co_north << std::endl;
    }
    else
    {
        std::cout << "Coordinate System north not set, exiting" << std::endl;
        //          << this->co_north << std::endl;
        std::raise(SIGINT);
    }

    this->agent->set_north(true_north_rad);

    if (vm.count("publish_frequency") + vm.count("publish_divisor") == 0) {
        std::cout << "must pass at least one of --publish_frequency/-f or --publish_divisor/-d" << std::endl;
        exit(1);
    }

    if (vm.count("publish_frequency")) {
        this->publish_frequency = vm["publish_frequency"].as<float>();
        if (this->publish_frequency <= 0.f) {
            std::cout << "publish_frequency must be greater than 0" << std::endl;
            exit(1);
        }
        std::cout << "If input data permets, publishing at " 
                  << this->publish_frequency << " Hz" << std::endl;
    } 

    if (vm.count("publish_divisor")) {
        this->publish_div = vm["publish_divisor"].as<unsigned int>();
        if (this->publish_div == 0) {
            std::cout << "publish_divisor must be greater than 0" << std::endl;
            exit(1);
        }
        std::cout << "Publishing every "
                  << this->publish_div << " th sample that will be received." << std::endl;
    }

    this->agent->set_publish_speed( this->publish_div, this->publish_frequency );

    if(vm.count("streaming_ids"))
    {
        this->streaming_ids = vm["streaming_ids"].as<std::vector<unsigned int>>();
        //std::cout << "Streaming IDs set to";
        //for(unsigned int id : this->streaming_ids) std::cout << " " << id << " ";
        //std::cout << std::endl;
    }
    else
    {
        std::cout << "Streaming IDs not set" <<std::endl;
        exit(1);
    }

    if(vm.count("craft_noses"))
    {
        this->craft_nose_strings = vm["craft_noses"].as<std::vector<std::string>>();
        if (this->craft_nose_strings.size() != this->streaming_ids.size()) {
            std::cout << "Streaming IDs input must be the same length as craft_noses!" << std::endl;
        }

        for (size_t i = 0; i < this->craft_nose_strings.size(); ++i) {
            std::string str = this->craft_nose_strings[i];
            boost::algorithm::to_lower(str);

            ArenaDirection dir;
            if(str.compare("right") == 0)
            {
                dir = ArenaDirection::RIGHT;
            }
            else if(str.compare("far_side") == 0)
            {
                dir = ArenaDirection::FAR_SIDE;
            }
            else if (str.compare("left") == 0)
            {
                dir = ArenaDirection::LEFT;
            }
            else if (str.compare("near_side") == 0)
            {
                dir = ArenaDirection::NEAR_SIDE;
            }
            else
            {
                std::cout << "A/C Nose Direction " << str << " not definied. Exiting" << std::endl;
                std::raise(SIGINT);
            }

            std::cout << "Creating rigid body with streaming id " << this->streaming_ids[i] << " and nose direction " << dir << std::endl;
            RigidBody* rb = new RigidBody( this->streaming_ids[i], dir );
            this->mocap->track_rigid_body(*rb);
        }
    }
    else
    {
        std::cout << "A/C nose direction not set, exiting" << std::endl;
        std::raise(SIGINT);
    }
}

void UnifiedMocapClient::banner()
{
    // generator and font: https://patorjk.com/software/taag/#p=display&f=Small&t=Type%20Something%20
    std::cout<< R"(
#################################################################################
##  _   _      _  __ _        _ __  __                    ___ _ _         _    ##
## | | | |_ _ (_)/ _(_)___ __| |  \/  |___  __ __ _ _ __ / __| (_)___ _ _| |_  ##
## | |_| | ' \| |  _| / -_) _` | |\/| / _ \/ _/ _` | '_ \ (__| | / -_) ' \  _| ##
##  \___/|_||_|_|_| |_\___\__,_|_|  |_\___/\__\__,_| .__/\___|_|_\___|_||_\__| ##
##                                                 |_|                         ##
#################################################################################)";
}

/*
void UnifiedMocapClient::print_coordinate_system() const
{
    // There's probably a better way to do this but I can't think
    // of it. So a bunch of nested switch-case statements it is.
    std::cout<< R"( 
               far
     +──────────────────────────+
     │                          │)";
    switch(this->craft_nose)
    {
        case ArenaDirection::RIGHT:
            std::cout << R"(
     │                          │
     │       [craft] ▶          │
     │                          │)";
            break;
        case ArenaDirection::FAR_SIDE:
            std::cout << R"(
     │            ▲             │
     │         [craft]          │
     │                          │)";
            break;
        case ArenaDirection::LEFT:
            std::cout << R"(
     │                          │
     │       ◀ [craft]          │
     │                          │)";
            break;
        case ArenaDirection::NEAR_SIDE:
            std::cout << R"(
     │                          │
     │         [craft]          │
     │            ▼             │)";
            break;
    }

    std::cout << R"(
left │                          │ right )";

    // If the up_axis is not detected the CO is not well defined
    // and we can't draw the coordinate system
    if(this->up_axis == UpAxis::NOTDETECTED)
    {
        std::cout << R"(
     │ UpAxis could not be      │
     │ detected. CO not well    │ 
     │ defined. Use a newer     │ 
     │ version of motive.       │)" << std::endl;
        return;
    }
    switch(this->co)
    {
        case CoordinateSystem::UNCHANGED:
            switch(this->up_axis)
            {
                case UpAxis::X:
                    std::cout << R"( 
     │      ↑                   │
     │    x ⊙ →                 │
     │                          │
     │    y-z Unchanged         │)";
                    break;
                case UpAxis::Y:
                    std::cout << R"( 
     │      ↑                   │
     │    y ⊙ →                 │
     │                          │
     │    x-z Unchanged         │)";
                    break;
                case UpAxis::Z:
                    std::cout << R"( 
     │      ↑                   │
     │    z ⊙ →                 │
     │                          │
     │    x-y Unchanged         │)";
                    break;
            }
            break;
        case CoordinateSystem::NED:
            switch (this->co_north)
            {
                case ArenaDirection::RIGHT:
                    std::cout << R"( 
     │                          │
     │   z  ⓧ → x               │
     │    y ↓                   │)";
                    break;
                case ArenaDirection::FAR_SIDE:
                    std::cout << R"( 
     │    x ↑                   │
     │   z  ⓧ → y               │
     │                          │)";
                    break;
                case ArenaDirection::LEFT:
                    std::cout << R"( 
     │    y ↑                   │
     │  x ← ⓧ z                 │
     │                          │)";
                    break;
                case ArenaDirection::NEAR_SIDE:
                    std::cout << R"( 
     │                          │
     │  y ← ⓧ z                 │
     │    x ↓                   │)";
                    break;
                case ArenaDirection::TRUE_NORTH: {
                    int rounded = static_cast<int>(std::round(this->true_north_deg));
                    char sign = (this->true_north_deg < 0) ? '-' : '+';
                    std::cout << R"( 
     │north )" << sign << std::setw(3) << std::setfill(' ') << std::abs(rounded) << R"(° from far side │)" << R"(
     │      ⓧ                   │
     │     z                    │)";
                    break;
                }
            }
            break;
        case CoordinateSystem::ENU:
            switch (this->co_north)
            {
                case ArenaDirection::FAR_SIDE:
                    std::cout << R"( 
     │    y ↑                   │
     │   z  ⊙ → x               │
     │                          │)";
                    break;
                case ArenaDirection::LEFT:
                    std::cout << R"( 
     │    x ↑                   │
     │  y ← ⊙ z                 │
     │                          │)";
                    break;
                case ArenaDirection::NEAR_SIDE:
                    std::cout << R"( 
     │                          │
     │  x ← ⊙ z                 │
     │    y ↓                   │)";
                    break;
                case ArenaDirection::RIGHT:
                    std::cout << R"( 
     │                          │
     │   z  ⊙ → y               │
     │    x ↓                   │)";
                    break;
                case ArenaDirection::TRUE_NORTH: {
                    int rounded = static_cast<int>(std::round(this->true_north_deg));
                    char sign = (this->true_north_deg < 0) ? '-' : '+';
                    std::cout << R"( 
     │north )" << sign << std::setw(3) << std::setfill(' ') << std::abs(rounded) << R"(° from far side │)" << R"(
     │      ⊙                   │
     │     z                    │)";
                    break;
                }
            }
            break;

    }
    
    std::cout<<R"(
     │                          │
     +──────────────────────────+
     │    Observers  (near)     │
     └──────────────────────────┘
     Assuming calibration triangle 
     long-edge has pointed to )" << this->long_edge << std::endl;
}
*/

// Function to simulate getch() in Linux
char getch(void) {
    struct termios oldt, newt;
    char ch;

    // Get the current terminal attributes
    tcgetattr(STDIN_FILENO, &oldt);
    
    // Set new terminal attributes for raw input
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read a single character
    ch = getchar();

    // Restore old terminal attributes
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}
