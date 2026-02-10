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

#include "mocap.hpp"
#include "pose_calculations.hpp"

Mocap::Mocap()
{
    // this->name = "mocap";
}

Mocap::Mocap(const Mocap &other)
{
    (void) other;
    std::cerr << "Copy constructor for Mocap not supported. Exiting." << std::endl;
    std::raise(SIGINT);
}

Mocap::~Mocap()
{
}

void Mocap::banner()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void Mocap::add_extra_po(boost::program_options::options_description &desc)
{
    (void)desc;
}

// Non-action implementation of the virtual function to make the implementation optional
void Mocap::parse_extra_po(const boost::program_options::variables_map &vm)
{
    (void)vm;
}

int Mocap::connect()
{
    return 1;
}

void Mocap::set_time_offset(uint64_t cam_time)
{
}

