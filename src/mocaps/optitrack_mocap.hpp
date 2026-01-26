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

#ifndef H_OPTITRACK_MOCAP
#define H_OPTITRACK_MOCAP

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm> // For std::find

#include <iostream>
#include <csignal>
#include <thread>
#include <inttypes.h>

#include <unistd.h>
#include <termios.h>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetRequests.h>

namespace po = boost::program_options;

enum UpAxis { NOTDETECTED=-1, X=0, Y, Z };

std::ostream& operator<<(std::ostream& lhs, ErrorCode e) {
    switch(e) {
    case ErrorCode_OK: lhs <<               "OK"; break;
    case ErrorCode_Internal: lhs <<         "Internal"; break;
    case ErrorCode_External: lhs <<         "External"; break;
    case ErrorCode_Network: lhs <<          "Network"; break;
    case ErrorCode_Other: lhs <<            "Other"; break;
    case ErrorCode_InvalidArgument: lhs <<  "InvalidArgument"; break;
    case ErrorCode_InvalidOperation: lhs << "InvalidOperation"; break;
    case ErrorCode_InvalidSize: lhs <<      "InvalidSize"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, UpAxis e) {
    switch(e) {
    case NOTDETECTED: lhs << "NOTDETECTED"; break;
    case X: lhs << "X"; break;
    case Y: lhs << "Y"; break;
    case Z: lhs << "Z"; break;
    }
    return lhs;
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);

class OptiTrackMocap : public Mocap
{
public:
    OptiTrackMocap()
    {
        // setup work, but as little as possible
        this->pClient = new NatNetClient();
        this->name = "optitrack";
    }

    ~OptiTrackMocap()
    {
        // cleanup if necessary
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout << R"(
##   ___       _   _ _____            _    ##
##  / _ \ _ __| |_(_)_   _| _ __ _ __| |__ ##
## | (_) | '_ \  _| | | || '_/ _` / _| / / ##
##  \___/| .__/\__|_| |_||_| \__,_\__|_\_\ ##
##       |_|                               ##
#############################################)";
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        // add extra commandlien options if you need to.
        // avoid those already used in UnifiedMocapRouter::read_po()
        //desc.add_options()
        //    ("dontmindme,d", boost::program_options::value<std::string>(), "Optimal extra argument for demonstration purposes")
        //    ("listofint,i", boost::program_options::value<std::vector<unsigned int>>()->multitopken(), "Optional list of values for demonstration purposes")
        //;
        desc.add_options()
            ("long_edge,l", po::value<std::string>(), "direction of long edge during Motive ground-plane calibration [right, far, left, near]")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if(vm.count("long_edge"))
        {
            //if (this->co == CoordinateSystem::UNCHANGED) {
            //    std::cout << "Can only specify --long_edge/-l when coordinate system is not UNCHANGED. Exiting" << std::endl;
            //    std::raise(SIGINT);
            //}
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
        ErrorCode ret;

        static constexpr unsigned int DISCOVERY_TIMEOUT = 1000;
        static constexpr unsigned int MAX_DISCOVERY = 10;
        std::cout<<std::endl<<"Discovering NatNet servers (timeout " << DISCOVERY_TIMEOUT << "ms)... ";

        int n = 1;
        sNatNetDiscoveredServer availableServers[MAX_DISCOVERY]; // just support one for now
        ret = NatNet_BroadcastServerDiscovery(availableServers, &n, DISCOVERY_TIMEOUT);
        if ((ret != ErrorCode_OK) || (n == 0)) {
            if (ret != ErrorCode_OK)
                std::cout << "Failed with Error code " << ret << std::endl;
            else {
                std::cout << "Failed: No servers found" << std::endl;
                ret = ErrorCode_Network;
            }

            std::cout<<std::endl<<"Troubleshooting: " << std::endl;
            std::cout<<"1. Verify connected to Motive network" << std::endl;
            std::cout<<"2. Verify that 'interface' is NOT set to 'loopback' in Motive 'Data Streaming Pane'" << std::endl;
            return ret;

        } else if (n > 1) {
            std::cout << "Failed: more than 1 server found:" << std::endl;
            for (int i=0; i<MAX_DISCOVERY; i++) {
                if (i >= n) { break; }
                std::cout << availableServers[i].serverAddress << std::endl;
            }
            std::cout << "Make sure you are connected by ethernet OR wi-fi, but not both" << std::endl;
            return ErrorCode_Network;

        }

        std::cout << "Successful!" << std::endl;

        this->connectParams.connectionType\
            = availableServers[0].serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
        this->connectParams.serverCommandPort = availableServers[0].serverCommandPort;
        this->connectParams.serverDataPort = availableServers[0].serverDescription.ConnectionDataPort;
        this->connectParams.serverAddress = availableServers[0].serverAddress;
        this->connectParams.localAddress = availableServers[0].localAddress;

        char mcastAddress[kNatNetIpv4AddrStrLenMax];
        snprintf(
            mcastAddress, sizeof mcastAddress,
            "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
            availableServers[0].serverDescription.ConnectionMulticastAddress[0],
            availableServers[0].serverDescription.ConnectionMulticastAddress[1],
            availableServers[0].serverDescription.ConnectionMulticastAddress[2],
            availableServers[0].serverDescription.ConnectionMulticastAddress[3]
        );
        this->connectParams.multicastAddress = mcastAddress;

        std::cout << std::endl << "Attempting connection to " << this->connectParams.serverAddress << "... ";
        ret = this->pClient->Connect(this->connectParams);
        if (ret == ErrorCode_OK) {
            std::cout<<"Successful!"<<std::endl;
        } else {
            std::cout<<"Failed with unknown error code "<< ret <<std::endl;
            return ret;
        }

        void* response;
        int nBytes;

        // detect host clock settings (for accurate time calcs and version detection)
        std::cout<<"Detecting Server Configuration.. ";
        memset( &(this->serverConfig), 0, sizeof( this->serverConfig ) );
        ret = this->pClient->GetServerDescription(&(this->serverConfig));
        if (ret == ErrorCode_OK) {
            std::cout << "Done" << std::endl;
        } else {
            std::cout << "Error code " << ret << std::endl;
            return ret;
        }

        // abort if unsupported NatNetVersion
        if (this->serverConfig.NatNetVersion[0] < 3) {
            std::cout << "ERROR: NatNet Version < 3 detected. Use Motive 2.x or newer"
                << std::endl;
            return ErrorCode::ErrorCode_External;
        }

        // detect up axis
        std::cout<<"Detecting up axis... ";
        ret = this->pClient->SendMessageAndWait("GetProperty,,Up Axis", &response, &nBytes);
        if (ret == ErrorCode_OK) {
            this->up_axis = static_cast<UpAxis>( ((char*)response)[0] - '0' );
            std::cout << this->up_axis << std::endl;
        } else {
            std::cout << "Error code " << ret << std::endl;
            return ret;
        }

        ret = this->pClient->SetFrameReceivedCallback( DataHandler, this );
        if (ret != ErrorCode_OK) {
            std::cout << "Registering frame received callback failed with Error Code " << ret << std::endl;
            return ret;
        }

        // inform user
        std::cout << std::endl << "INFO: if you see this message but you still don't receive messages, check:" << std::endl;
        std::cout << "1. Rigid body streaming id(s) are correct" << std::endl;
        std::cout << "2. Rigid body(s) are selected in Motive" << std::endl;
        std::cout << "3. 'Multicast' is selected in 'Data Streaming Pane' in Motive" << std::endl;

        return ErrorCode_OK;
    }

    void set_time_offset(uint64_t cam_time) override 
    {
        this->agent->set_time_offset(this->pClient->SecondsSinceHostTimestamp(cam_time));
    }

    void data_handler(sFrameOfMocapData* data)
    {
        // get timestamp
        uint64_t timeAtExpoUs = data->CameraMidExposureTimestamp / (this->serverConfig.HighResClockFrequency * 1e-6);

        // loop over bodies in frame and process the ones we listen to
        bool anyTracked = false;
	    for (int i=0; i < data->nRigidBodies; i++) {
            unsigned int rb_id = data->RigidBodies[i].ID;

            RigidBody* theRb = nullptr;
            for (auto& rb : this->RBs) {
                if (rb.streaming_id == rb_id) {
                    theRb = &rb;
                    break;
                }
            }

            if (!theRb) {
                continue; // untracked by us
            }

            bool bTrackingValid = data->RigidBodies[i].params & 0x01;
            if (!bTrackingValid)
                continue;

            anyTracked = true;

            pose_t newPose {
                timeAtExpoUs,
                data->RigidBodies[i].x,
                data->RigidBodies[i].y,
                data->RigidBodies[i].z,
                data->RigidBodies[i].qx,
                data->RigidBodies[i].qy,
                data->RigidBodies[i].qz,
                data->RigidBodies[i].qw,
            };

            pose_t newPoseYUP = newPose;

            // TODO: convert!!!
            switch(this->up_axis)
            {
                case UpAxis::X:
                    // Transform from X-Up to Y-Up
                    newPoseYUP.x = -newPose.y;
                    newPoseYUP.y = newPose.x;
                    newPoseYUP.z = newPose.z;

                    newPoseYUP.qw = newPose.qw;
                    newPoseYUP.qx = -newPose.qy;
                    newPoseYUP.qy = newPose.qx;
                    newPoseYUP.qz = newPose.qz;
                    break;
                case UpAxis::Y:
                    // We do nothing because this is what we want to have
                    break;
                case UpAxis::Z:
                    // Transform from X-Up to Y-Up
                    newPoseYUP.x = newPose.x;
                    newPoseYUP.y = newPose.z;
                    newPoseYUP.z = -newPose.y;

                    newPoseYUP.qw = newPose.qw;
                    newPoseYUP.qx = newPose.qx;
                    newPoseYUP.qy = newPose.qz;
                    newPoseYUP.qz = -newPose.qy;
                    break;
                case UpAxis::NOTDETECTED:
                default:
                    // The up axis is not known. Abort
                    std::cerr << "The up-axis is not detected. Aborting." << std::endl;
                    std::raise(SIGINT);
                    break;
            }

            // convert yup to ENU with east being the long edge of the calib triangle
            pose_t newPoseENU_longEdgeEast = {
                timeAtExpoUs,
                .x = newPoseYUP.z, newPoseYUP.x, newPoseYUP.y,
                .qx = newPoseYUP.qz, .qy = newPoseYUP.qx, .qz = newPoseYUP.qy, .qw = newPoseYUP.qw,
            };

            pose_t newPoseENU_northFarSide = newPoseENU_longEdgeEast;

            switch(long_edge)
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
            std::cout << "Received NatNet data for " << data->nRigidBodies << " rigid bodies for host time: " << timeAtExpoUs << "us";
            //if (!anyTracked) {
                std::cout << ", but none are tracked";
            //}
            std::cout << std::endl;
        }

        if (anyTracked) {
            // internally, we still check if there _actually_ is a new sample
            // for each rigid body
            this->agent->new_data_available( this->RBs );
        }

        return;
    }

private:
    NatNetClient* pClient;
    sNatNetClientConnectParams connectParams;
    sServerDescription serverConfig;
    UpAxis up_axis;
    CoordinateSystem co;
    ArenaDirection long_edge;
};

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData) {
    OptiTrackMocap* that = (OptiTrackMocap*) pUserData;
    that->data_handler(data);

    // Update time delay to agent
    that->set_time_offset(data->CameraMidExposureTimestamp);
};

#endif // H_OPTITRACK_MOCAP