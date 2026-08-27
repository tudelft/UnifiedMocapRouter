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

#ifndef H_VICON_MOCAP
#define H_VICON_MOCAP

#include <vector>
#include <string>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <math.h>

#include <unistd.h>

#include "DataStreamClient.h"

#include "pose_calculations.hpp"


namespace po = boost::program_options;
namespace VDS = ViconDataStreamSDK::CPP;


class ViconMocap : public Mocap
{
private:
    std::string server_addr;
    ArenaDirection long_edge;

    VDS::Client client;

    std::thread receiverThread;
    bool shouldStop;

public:
    ViconMocap() : Mocap{}, long_edge{ArenaDirection::RIGHT}, shouldStop{false}
    {
        // setup work, but do as little as possible
        this->name = "vicon";
    }

    ~ViconMocap()
    {
        // cleanup if necessary
        this->shouldStop = true;
        if (this->receiverThread.joinable()) {
            this->receiverThread.join();
        }
        this->client.DisableSegmentData();
        this->client.Disconnect();
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##  __   ___                 ##
##  \ \ / (_)__ ___ _ _      ##
##   \ V /| / _/ _ \ ' \    ##
##    \_/ |_\__\___/_||_|    ##
##############################)";
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("mocap_ip", boost::program_options::value<std::string>(), "Vicon DataStream server IP (Vicon Tracker host).")
            ("long_edge,l", po::value<std::string>(), "direction of long edge during ground-plane calibration [right, far, left, near]")
            ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("mocap_ip")) {
            std::string val = vm["mocap_ip"].as<std::string>();
            std::cout << "Vicon server ip " << val << std::endl;
            this->server_addr = val;
        } else {
            std::cout << "No Vicon server ip passed" << std::endl;
            std::raise(SIGINT);
        }

        if(vm.count("long_edge"))
        {
            std::string le = vm["long_edge"].as<std::string>();
            boost::algorithm::to_lower(le);

            if(le.compare("right") == 0)
            {
                this->long_edge = ArenaDirection::RIGHT;
            }
            else if(le.compare("far") == 0)
            {
                this->long_edge = ArenaDirection::FAR;
            }
            else if (le.compare("left") == 0)
            {
                this->long_edge = ArenaDirection::LEFT;
            }
            else if (le.compare("near") == 0)
            {
                this->long_edge = ArenaDirection::NEAR;
            }
            else
            {
                std::cout << "Long Edge Direction " << le << " not definied. Exiting" << std::endl;
                std::raise(SIGINT);
            }
            std::cout << "Long Edge direction set to " << this->long_edge << std::endl;
        }
        else
        {
            std::cout << "Long Edge direction not set, defaulting to "
                      << this->long_edge << std::endl;
        }
    }

    int connect() override
    {
        // will be called the first thing after all options are parsed
        // should be used to establish connection with the MOCAP system, and
        // configure it if necessary

        // IMPORTANT:
        // every time a new sample comes in, the base class method
        // "new_data_available" must be invoked. Here we do this in the
        // data_handler thread, which polls the Vicon DataStream.

        // Vicon Connect() returns immediately; retry a bounded number of
        // times until the connection is actually established.
        unsigned int attempts = 0;
        const unsigned int maxAttempts = 30;
        while (!this->client.IsConnected().Connected) {
            if (attempts++ >= maxAttempts) {
                std::cout << "Failed to connect to Vicon server at "
                          << this->server_addr << " after " << maxAttempts
                          << " attempts." << std::endl;
                std::raise(SIGINT);
            }
            if (this->client.Connect(this->server_addr).Result != VDS::Result::Success) {
                std::cout << "Vicon connection attempt failed, retrying..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::cout << "Connected to Vicon server at " << this->server_addr << std::endl;

        // We only need the rigid-body (segment) poses.
        this->client.EnableSegmentData();

        // ServerPush: the server pushes every new frame; GetFrame() blocks
        // until one is available. Lowest-latency mode (cf. Qualisys StreamFrames).
        this->client.SetStreamMode(VDS::StreamMode::ServerPush);

        // Offload the up-axis convention to the server: Z-up, right-handed ENU
        // (X forward, Y left, Z up). The remaining yaw alignment to the
        // north-far-side frame is done with the long_edge switch below,
        // identically to the Qualisys backend.
        this->client.SetAxisMapping(VDS::Direction::Forward,
                                    VDS::Direction::Left,
                                    VDS::Direction::Up);

        this->receiverThread = std::thread(&ViconMocap::data_handler, this);

        return 0;
    }

    void data_handler() {
        while (!this->shouldStop) {
            // Block until the next frame is pushed by the server.
            if (this->client.GetFrame().Result != VDS::Result::Success) {
                continue;
            }

            // Vicon does not hand us a marker timestamp through the segment
            // API, so tag the sample with the host reception time, like the
            // test backend.
            auto now = std::chrono::high_resolution_clock::now();
            uint64_t markerTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                        now.time_since_epoch()).count();

            bool anyTracked = false;
            const unsigned int subjectCount = this->client.GetSubjectCount().SubjectCount;

            for (unsigned int i = 0; i < subjectCount; i++)
            {
                std::string subjectName = this->client.GetSubjectName(i).SubjectName;

                RigidBody* theRb = nullptr;
                for (auto& rb : this->RBs) {
                    if (subjectName.compare(rb.streaming_name) == 0) {
                        theRb = &rb;
                        break;
                    }
                }

                if (!theRb) {
                    continue; // untracked
                }

                // The root segment carries the global pose of the subject.
                std::string rootSegment = this->client.GetSubjectRootSegmentName(subjectName).SegmentName;

                VDS::Output_GetSegmentGlobalTranslation trans =
                    this->client.GetSegmentGlobalTranslation(subjectName, rootSegment);
                VDS::Output_GetSegmentGlobalRotationQuaternion rot =
                    this->client.GetSegmentGlobalRotationQuaternion(subjectName, rootSegment);

                if (trans.Result != VDS::Result::Success
                    || rot.Result != VDS::Result::Success
                    || trans.Occluded || rot.Occluded) {
                    continue;
                }

                if ( isnan(trans.Translation[0]) || isnan(trans.Translation[1]) || isnan(trans.Translation[2]) ) {
                    continue;
                }

                anyTracked = true;

                // Vicon translations are in millimeters; quaternion is [x,y,z,w].
                pose_t newPoseENU_longEdgeEast = {
                    markerTimeUs,
                    .x = 1e-3f * (float)trans.Translation[0],
                    .y = 1e-3f * (float)trans.Translation[1],
                    .z = 1e-3f * (float)trans.Translation[2],
                    .qx = (float)rot.Rotation[0],
                    .qy = (float)rot.Rotation[1],
                    .qz = (float)rot.Rotation[2],
                    .qw = (float)rot.Rotation[3]
                };

                pose_t newPoseENU_northFarSide = newPoseENU_longEdgeEast;

                // long edge means x to the right (identical convention to Qualisys)
                switch(this->long_edge)
                {
                    case ArenaDirection::RIGHT:
                        // We do nothing because this is what we want to have
                        break;
                    case ArenaDirection::FAR:
                        // Rotate to align in the yaw plane
                        newPoseENU_northFarSide.y = newPoseENU_longEdgeEast.x;
                        newPoseENU_northFarSide.x = -newPoseENU_longEdgeEast.y;

                        newPoseENU_northFarSide.qy = newPoseENU_longEdgeEast.qx;
                        newPoseENU_northFarSide.qx = -newPoseENU_longEdgeEast.qy;
                        break;
                    case ArenaDirection::LEFT:
                        // Rotate to align in the yaw plane
                        newPoseENU_northFarSide.y = -newPoseENU_longEdgeEast.y;
                        newPoseENU_northFarSide.x = -newPoseENU_longEdgeEast.x;

                        newPoseENU_northFarSide.qy = -newPoseENU_longEdgeEast.qy;
                        newPoseENU_northFarSide.qx = -newPoseENU_longEdgeEast.qx;
                        break;
                    case ArenaDirection::NEAR:
                        // Rotate to align in the yaw plane
                        newPoseENU_northFarSide.y = -newPoseENU_longEdgeEast.x;
                        newPoseENU_northFarSide.x = newPoseENU_longEdgeEast.y;

                        newPoseENU_northFarSide.qy = -newPoseENU_longEdgeEast.qx;
                        newPoseENU_northFarSide.qx = newPoseENU_longEdgeEast.qy;
                        break;
                }

                theRb->setNewPoseENU_NorthFarSide( newPoseENU_northFarSide );
            }

            if ( this->agent->printMessages && !anyTracked ) {
                std::cout << "Received Vicon frame with " << subjectCount
                          << " subjects for host time: " << markerTimeUs
                          << "us, but none are tracked" << std::endl;
            }

            if (anyTracked) {
                // internally, we still check if there _actually_ is a new sample
                // for each rigid body
                this->agent->new_data_available( this->RBs );
            }
        }
    }
};
#endif // H_VICON_MOCAP
