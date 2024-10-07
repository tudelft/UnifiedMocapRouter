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

#ifndef H_QUALISYS_SDK_MOCAP
#define H_QUALISYS_SDK_MOCAP

#include <vector>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>

#include <unistd.h>

#include "RTPacket.h"
#include "RTProtocol.h"

#include "pose_calculations.hpp"


namespace po = boost::program_options;

//using namespace boost::asio;

enum QualisysType {
    Error = 0,
    Command,
    XML,
    Data,
    NoMoreData,
    C3Cfile,
    Event,
    Discover,
    QTMfile
};

enum QualisysComponent {
    _6D = 5
};

// todo: gcc packing?
typedef struct qualisys_header_s {
    unsigned int Size;
    unsigned int Type;
} qualisys_header_t;

typedef struct qualisys_packet_s {
    qualisys_header_t header;
    char* data;
} qualisys_packet_t;

// udp streaming packet header
typedef struct qualisys_data_header_s {
    unsigned int Size;
    unsigned int Type;
    unsigned long long MarkerTimestamp;
    unsigned int MarkerFrameNumber;
    unsigned int ComponentCount;
} qualisys_data_header_t;

// 6DOF components
typedef struct qualisys_6dof_header_s {
    unsigned int ComponentSize;
    unsigned int ComponentType;
    unsigned int BodyCount;
    unsigned short dropRate;
    unsigned short outOfSyncRate;
} qualisys_6dof_header_t;

typedef struct qualisys_6dof_body_s {
    float x;  // position
    float y;
    float z;
    float r00; // rotation matrix
    float r01;
    float r02;
    float r10;
    float r11;
    float r12;
    float r20;
    float r21;
    float r22;
} qualisys_6dof_body_t;



class QualisysSdkMocap : public Mocap
{
private:
    std::string server_addr;
    unsigned short int base_port;
    unsigned short int stream_port;
    bool bigEndian;
    CRTProtocol rtProtocol;
    
    std::thread receiverThread;
    bool shouldStop;

public:
    QualisysSdkMocap() : Mocap{}, base_port{22222}, stream_port{12345}, bigEndian{false}, shouldStop{false}
    {
        // setup work, but do as little as possible
    }

    ~QualisysSdkMocap()
    {
        // cleanup if necessary
        this->shouldStop = true;
        this->receiverThread.join();
        this->rtProtocol.StopCapture();
        this->rtProtocol.Disconnect();
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##   ___            _ _             ##
##  / _ \ _  _ __ _| (_)____  _ ___ ##
## | (_) | || / _` | | (_-< || (_-< ##
##  \__\_\\_,_\__,_|_|_/__/\_, /__/ ##
##                         |__/     ##
######################################)";
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("mocap_ip", boost::program_options::value<std::string>(), "QTM IP.")
            ("mocap_base_port", boost::program_options::value<unsigned short int>(), "QTM TCP base port.")
            ("mocap_stream_port", boost::program_options::value<unsigned short int>(), "Arbitrary UDP port 1024-65535 for streaming.")
            ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("mocap_ip")) {
            std::string val = vm["mocap_ip"].as<std::string>();
            std::cout << "QTM server ip " << val << std::endl;
            this->server_addr = val;
        } else {
            std::cout << "No QTM server ip passed" << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("mocap_tcp_port")) {
            unsigned short int val = vm["mocap_base_port"].as<unsigned short int>();
            std::cout << "Connecting to QTM using TCP base port " << val << std::endl;
            this->base_port = val;
        } else {
            std::cout << "No QTM TCP port given. Assume 22222" << std::endl;
            this->base_port = 22222;
        }

        if (vm.count("mocap_stream_port")) {
            unsigned short int val = vm["mocap_stream_port"].as<unsigned short int>();
            std::cout << "Telling QTM to stream on UDP port " << val << std::endl;
            this->stream_port = val;
        } else {
            std::cout << "No QTM udp port given. Assume 12345" << std::endl;
            this->stream_port = 12345;
        }
    }

    int connect() override
    {
        // will be called the first thing after all options are parsed
        // should be used to establish connection with the MOCAP system, and
        // configure it if necessary

        // IMPORTANT: 
        // every time a new sample comes in, the base class method 
        // "processNewPose" must be invoked.
        // in this test example, we do this at the end of the send_test_data
        // thread.
        // also see the optitrack client, which does it with callback

        const int majorVersion = 1;
        const int minorVersion = 24; // todo: double check this
        if (!this->rtProtocol.Connect(this->server_addr.c_str(), this->base_port, &this->stream_port, majorVersion, minorVersion, this->bigEndian))
        {
            std::cout << "rtProtocol.Connect: " << this->rtProtocol.GetErrorString() << std::endl;
            std::raise(SIGINT);
        }

        bool data_available = false;
        if (!this->rtProtocol.Read6DOFSettings(data_available))
        {
            //RCLCPP_ERROR(this->get_logger(), "rtProtocol.Read6DOFSettings: %s", rtProtocol.GetErrorString());
            std::cout << "error in read 6dof settings" << std::endl;
            sleep(1);
            std::raise(SIGINT);
        }

        // NULL: just send back to the host that sends the request
        // cComponent6D: position and rotation matrix
        if (!this->rtProtocol.StreamFrames(CRTProtocol::RateFrequencyDivisor, 1, this->stream_port, NULL, CRTProtocol::cComponent6d))
        {
            std::cout << "rtProtocol.StreamFrames: " << rtProtocol.GetErrorString() << std::endl;
            std::raise(SIGINT);
        }

        this->receiverThread = std::thread(&QualisysSdkMocap::data_handler,
                                    this
                                    );

        return 0;
    }

    void data_handler() {
        while (!this->shouldStop) {
            bool anyTracked = false;

            CRTPacket::EPacketType packetType;
            if (rtProtocol.Receive(packetType, true) == CNetwork::ResponseType::success)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rmatVec[9];

                    CRTPacket* rtPacket = rtProtocol.GetRTPacket();
                    uint64_t markerTimeUs = rtPacket->GetTimeStamp();

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rmatVec))
                        {
                            const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
                            RigidBody* theRb = nullptr;
                            if (pTmpStr) {
                                for (auto& rb : this->RBs) {
                                    if (strcmp(pTmpStr, rb.name.c_str()) == 0) {
                                        theRb = &rb;
                                        break;
                                    }
                                }
                            } else {
                                // untracked
                                continue;
                            }

                            if (!theRb) {
                                continue; // untracked
                            }

                            anyTracked = true;

                            // TODO: check rotation matrix transpose
                            quaternion_t quat;
                            rotationMatrix_t rotM = { .m = {
                                    { rmatVec[0], rmatVec[3], rmatVec[6] },
                                    { rmatVec[1], rmatVec[4], rmatVec[7] },
                                    { rmatVec[2], rmatVec[5], rmatVec[8] }
                            }};

                            quaternion_of_rotationMatrix(&quat, &rotM);

                            pose_t newPose {
                                markerTimeUs,
                                .x = 1e-3f * fX, .y = 1e-3f * fY, .z = 1e-3f * fZ,
                                .qx = quat.x, .qy = quat.y, .qz = quat.z, .qw = quat.w
                            };

                            // TODO; convert transform

                            theRb->setNewPoseENU_NorthFarSide( newPose );
                        }
                    }

                    if ( this->agent->printMessages && !anyTracked ) {
                        std::cout << "Received QTM RT data for " << rtPacket->Get6DOFBodyCount() << " 6DOF rigid bodies for host time: " << markerTimeUs << "us";
                        std::cout << ", but none are tracked";
                        std::cout << std::endl;
                    }

                    if (anyTracked) {
                        // internally, we still check if there _actually_ is a new sample
                        // for each rigid body
                        this->agent->new_data_available( this->RBs );
                    }
                }
            }
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

/*
    void data_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
    {
        char* buf = recv_buffer.data();

        qualisys_data_header_t header;
        memcpy(&header, buf, sizeof(qualisys_data_header_t));
        buf += sizeof(qualisys_data_header_t);

        if (header.Type != QualisysType::Data) { goto bail_out; }

        for (int i=0; i < header.ComponentCount; i++) {
            //std::cout << i << std::endl;
            qualisys_6dof_header_t header6d;
            memcpy(&header6d, buf, sizeof(qualisys_6dof_header_t));
            buf += sizeof(qualisys_6dof_header_t);

            if (header6d.ComponentType != QualisysComponent::_6D) {
                continue;
            }

            for (int j = 0; j < header6d.BodyCount; j++) {
                qualisys_6dof_body_t body;
                memcpy(&body, buf, sizeof(qualisys_6dof_body_t));
                buf += sizeof(qualisys_6dof_body_t);

                quaternion_t quat;
                rotationMatrix_t rotM = { .m = {
                        { body.r00, body.r01, body.r02 },
                        { body.r10, body.r11, body.r12 },
                        { body.r20, body.r21, body.r22 },
                }};

                quaternion_of_rotationMatrix(&quat, &rotM);

                pose_t newPose = { header.MarkerTimestamp,
                    body.x, body.y, body.z,
                    quat.x, quat.y, quat.z, quat.w
                    };

                if (this->RBs.size() > 0) {
                    this->RBs[0].setNewPoseENU_NorthFarSide( newPose );
                }

                // only the first body gets read
                i = header.ComponentCount;
                break;
            }
        }

        this->agent->new_data_available( this->RBs );

bail_out:
        //std::cout << header.Type << std::endl;
        wait();
    }

    void wait() {
        this->socket_udp.async_receive_from(boost::asio::buffer(this->recv_buffer),
            this->remote_endpoint,
            boost::bind(&QualisysMocap::data_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }


    void send_tcp_command(const std::string& command)
    {
        qualisys_header_t header;
        header.Size = command.length() + sizeof(qualisys_header_t);
        header.Type = QualisysType::Command;

        write(*this->socket_tcp, boost::asio::buffer(&header, sizeof(qualisys_header_t)));
        write(*this->socket_tcp, boost::asio::buffer(command));

        std::cout << "Sent command: " << command << std::endl;
    }
*/
};

#endif // H_QUALISYS_MOCAP