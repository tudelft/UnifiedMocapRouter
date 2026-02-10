// Copyright 2024 Till Blaha (Delft University of Technology)
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

#ifndef H_MOCAP
#define H_MOCAP

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include "pose_calculations.hpp"
#include "rigid_body.hpp"
#include "agent.hpp"

class Mocap
{
protected:
    Agent* agent;
    std::vector<RigidBody> RBs;

public:
    Mocap();
    Mocap(const Mocap &other);
    ~Mocap();
    void enroll_agent(Agent *a) {
        this->agent = a;
    }
    void track_rigid_body( RigidBody& rb ) {
        this->RBs.push_back(rb);
    }

    std::string name; // Name of the mocap system printed in --quiet mode

    virtual void banner();

    // Extra Program Options
    virtual void add_extra_po(boost::program_options::options_description &desc);
    virtual void parse_extra_po(const boost::program_options::variables_map &vm);
    virtual int connect();

    virtual void set_time_offset(uint64_t cam_time);
};

#endif