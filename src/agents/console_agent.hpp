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

#ifndef CONSOLE_AGENT_HPP
#define CONSOLE_AGENT_HPP

#include "agent.hpp"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

class ConsoleAgent : public Agent
{
public:
    ConsoleAgent()
    {
        // do as little setup here as possible. Use pre_start instead
        this->name = "console";
    }

    ~ConsoleAgent()
    {
        // put cleanup here if needed
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##   ___                  _      ##
##  / __|___ _ _  ___ ___| |___  ##
## | (__/ _ \ ' \(_-</ _ \ / -_) ##
##  \___\___/_||_/__/\___/_\___| ##
###################################)" << std::endl;
    }

private:
    void add_extra_po(boost::program_options::options_description &desc) override
    {
        // add extra commandlien options if you need to.
        // avoid those already used in UnifiedMocapRouter::read_po()
        //desc.add_options()
        //    ("dontmindme", boost::program_options::value<std::string>(), "Optimal extra argument for demonstration purposes")
        //    ("listofint", boost::program_options::value<std::vector<unsigned int>>()->multitopken(), "Optional list of values for demonstration purposes")
        //;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        /*
        if (vm.count("dontmindme")) {
            // value can be accessed with vm["dontmindme"].as<std::string>();
            std::cout << "Give user some feedback about the chosen" << vm["dontmindme"].as<std::string>() << std::endl;
        } else {
            // fail if not found, or use some default value
            std::cout << "dontmindme not specified. aborting..." << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("listofint")) {
            // values can be accessed via vm["listofint"].as<std::vector<unsigned int>>();
        } else {
            // do something
        }
        */
    }

    void pre_start() override
    {
        // gets called after:
        // 1. UnifiedMocapRouter is constructed
        // 2. this class is constructed
        // 3. arguments are parsed
        // 4. MocapClient and its callback handlers are constructed

        // do your setup work here, like initializing ports, opening lopfiles, etc
        this->initialized = true;
    }

    /*
    void post_start() override
    {
        // this gets called after publish_loop thread is spun
        // must be blocking!!
    }
    */

    bool publish_data(int idx, pose_t& pose, twist_t& twist) override
    {
        // this gets called every this->publish_dt seconds on all bodies to be 
        // published. Implement your sending routine here.
        // for instance, this is how to just print the z-position of all tracked bodies

        // don't run anything expensive in here, just publishing

        /*
        std::cout << "Rigid body with streaming id " << streaming_id 
                    << " has z position " << pose->z << std::endl;
        }
        */
       return true;
    }

private:
    // can be used for extra class members
};

#endif // ifndef CONSOLE_AGENT_HPP
