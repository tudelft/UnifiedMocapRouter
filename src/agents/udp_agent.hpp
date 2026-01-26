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

#ifndef UDP_AGENT_HPP
#define UDP_AGENT_HPP

#include "agent.hpp"

#include <unistd.h>
#include <csignal>
#include "boost/asio.hpp"

using namespace boost::asio;

class UdpAgent : public Agent
{
public:
    UdpAgent() : _socket{_io_service}
    {
        // do as little setup here as possible. Use pre_start instead
        this->name = "udp";
    }

    ~UdpAgent()
    {
        this->_socket.close();
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##  _  _  __   __   ##
## | | | |   \| _ \ ##
## | |_| | |) |  _/ ##
##  \___/|___/|_|   ##
######################)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("client_ip,i", boost::program_options::value<std::string>(), "IP to stream the UDP data to.")
            ("port,p", boost::program_options::value<unsigned short int>(), "UDP Port.")
            ("ac_id,ac", boost::program_options::value<std::vector<unsigned int>>()->multitoken(), "Optional Aircraft ID corresponding to the rigid body id(s). These ids will be included in the UDP messages")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("client_ip")) {
            std::string val = vm["client_ip"].as<std::string>();
            std::cout << "UDP client ip " << val << std::endl;
            this->_client_ip = val;
        } else {
            std::cout << "No UDP client ip passed" << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("port")) {
            unsigned short int val = vm["port"].as<unsigned short int>();
            std::cout << "Streaming UDP on port " << val << std::endl;
            this->_port = val;
        } else {
            std::cout << "No UDP port given. Assume 5005" << std::endl;
            this->_port = 5005;
        }

        if (this->streaming_ids.size() > 1) {
            std::cout << "Number of streaming_ids and ac_ids must be equal to 1 for the udp client. Multiple not (yet) supported"
                << std::endl;
            std::raise(SIGINT);
        }

        if(vm.count("ac_id")) {
            this->_ac_id = vm["ac_id"].as<std::vector<unsigned int>>();
            if ( this->_ac_id.size() != this->streaming_ids.size() ) {
                std::cout << "Number of ac_ids must be equal to streaming_ids" << std::endl;
                std::raise(SIGINT);
            }
            std::cout << "AC IDs set to";
            for(unsigned int id : this->_ac_id) std::cout << " " << id << " ";
            std::cout << std::endl;
        } else {
            this->_ac_id.push_back(0);
            std::cout << "No ac_id passed. Assuming 0." << std::endl;
        }
    }

    void pre_start() override
    {
        this->_remote_endpoint = ip::udp::endpoint(ip::address::from_string(this->_client_ip), this->_port);
        this->_socket.open(ip::udp::v4());
        this->initialized = true;
    }

    bool publish_data(int idx, pose_t& pose, twist_t& twist) override
    {
        static constexpr size_t Ni = sizeof(this->_ac_id[0]);
        static constexpr size_t Np = sizeof(pose_t);
        static constexpr size_t Nd = sizeof(twist_t);

        uint8_t buf[Ni+Np+Nd];
        memcpy(buf, &(this->_ac_id[idx]), Ni);
        memcpy(buf+Ni, &pose, Np);
        memcpy(buf+Ni+Np, &twist, Nd);

        boost::system::error_code err;
        auto sent = this->_socket.send_to(buffer(buf, Ni+Np+Nd), this->_remote_endpoint, 0, err);
        if (err.failed()) {
            std::cout << "Failed with error " << err << std::endl;
            return false;
        }
        return true;
    }

private:
    std::vector<unsigned int> _ac_id;
    unsigned short int _port;
    std::string _client_ip;
    io_service _io_service;
    ip::udp::socket _socket;
    ip::udp::endpoint _remote_endpoint;
};

#endif // ifndef UDP_AGENT_HPP
