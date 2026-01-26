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

#include "agent.hpp"
#include "pose_calculations.hpp"

Agent::Agent() : printMessages{false}, publish_every{1}, csys{CoordinateSystem::NED}, publish_duration{0}
{
    // this->name = "agent";
}

Agent::Agent(const Agent &other)
{
    (void) other;
    std::cerr << "Copy constructor for Agent not supported. Exiting." << std::endl;
    std::raise(SIGINT);
}

Agent::~Agent()
{
}

void Agent::banner()
{
}


void Agent::register_rigid_body(const RigidBody& RB)
{
    this->streaming_ids.push_back(RB.streaming_id);
    this->streaming_names.push_back(RB.streaming_name);
}

void Agent::set_publish_speed( unsigned int div, float freq ) {
    this->publish_every = div;
    this->publish_frequency = freq;
}

void Agent::set_csys( CoordinateSystem csys ) {
    this->csys = csys;
}

void Agent::set_north( float true_north_rad ) {
    this->true_north_rad = true_north_rad;
}

void Agent::set_time_offset(float time_offset) {
    this->time_offset = time_offset;
}

void Agent::new_data_available( std::vector<RigidBody>& RBs ) {
    if(!initialized) return;

    for (size_t i = 0; i < RBs.size(); ++i) {
        bool divisorReady = ( this->publish_every > 0 )  // true if divisor cli argument was passed
            && (RBs[i].getNumUnpublishedSamples() >= this->publish_every);
        bool hasUnpublishedSample = ( RBs[i].getNumUnpublishedSamples() > 0 );
        bool frequencyReady = ( this->publish_frequency > 0.f )  // true if frequency cli argument was passed
            && ( (RBs[i].getLatestSampleTime() - RBs[i].getLastPublishedSampleTime()) 
                // Subtract the publishing duration to more closely achieve the desired publishing duration.
                // Needs to be set in the respective agent. 
                  >= ((uint64_t) 1e6*(1.f / this->publish_frequency + this->publish_duration)) );

        if (divisorReady || (hasUnpublishedSample && frequencyReady)) {
            pose_t pose = RBs[i].getPoseIn( this->csys, this->true_north_rad );
            twist_t twist = RBs[i].getTwistIn( this->csys, this->true_north_rad );
            this->publish_data(
                i,
                pose,
                twist
            );
            if ( this->printMessages ) {
                this->print_data(
                    i,
                    pose,
                    twist
                );
            }
            RBs[i].setPublished();
        }
    }
}

void Agent::print_data(int idx, pose_t& pose, twist_t& twist)
{
    //printf("Incoming Rigid Body Data Frame [ID=%d Error=%3.4f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
    printf("\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    //printf("Incoming: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
    //    data->RigidBodies[i].x,
    //    data->RigidBodies[i].y,
    //    data->RigidBodies[i].z,
    //    data->RigidBodies[i].qx,
    //    data->RigidBodies[i].qy,
    //    data->RigidBodies[i].qz,
    //    data->RigidBodies[i].qw);
    printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
        pose.x,
        pose.y,
        pose.z,
        pose.qx,
        pose.qy,
        pose.qz,
        pose.qw);
    printf("\t\tvx\tvy\tvz\twx\twy\twz\n");
    printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n\n",
        twist.vx,
        twist.vy,
        twist.vz,
        twist.wx,
        twist.wy,
        twist.wz);
}

// Non-action implementation of the virtual function to make the implementation optional
bool Agent::publish_data(int idx, pose_t& pose, twist_t& twist)
{
    return false;
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::pre_start()
{
    this->initialized = true;
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::post_start()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::add_extra_po(boost::program_options::options_description &desc)
{
    (void)desc;
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::parse_extra_po(const boost::program_options::variables_map &vm)
{
    (void)vm;
}

