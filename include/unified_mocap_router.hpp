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

#ifndef H_UNIFIED_OPTITRACK_ROUTER
#define H_UNIFIED_OPTITRACK_ROUTER

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include "mocap.hpp"
#include "agent.hpp"

#include "pose_calculations.hpp"

char getch(void);

class UnifiedMocapRouter
{
private:
    Mocap* mocap;
    Agent* agent;
    bool printMessages;
    bool quiet;
    float publish_frequency;
    unsigned int publish_div;

    std::vector<unsigned int> streaming_ids;
    std::vector<std::string> streaming_names;
    std::vector<std::string> craft_nose_strings;

    void banner();
    void add_base_po();
    void parse_base_po(int argc, char const *argv[]);
    //void print_coordinate_system() const;

    std::thread keyThread;

    void keystroke_loop();

public:
    UnifiedMocapRouter(Mocap* mocap, Agent* agent);
    UnifiedMocapRouter(const UnifiedMocapRouter &other);
    ~UnifiedMocapRouter() {};

    /* Function that needs to be called after creating
     * this object, to start everything */
    void start(int argc, char const *argv[]);

    boost::program_options::options_description desc;
    boost::program_options::variables_map vm;
};


#endif //H_UNIFIED_OPTITRACK_ROUTER