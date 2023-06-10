/*
 * MIT License
 * 
 * Copyright (c) 2023 MiYA LAB(K.Miyauchi)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

//-----------------------------
// include
//-----------------------------
// STL
#include <string>
#include <memory>

// Boost
#include <boost/array.hpp>
#include <boost/asio/ip/udp.hpp>

// MiYALAB
#include "MiYALAB/Sensor/RFansLiDAR/driver.hpp"

//-----------------------------
// Namespace & using
//-----------------------------
using namespace boost::asio;
using namespace boost::asio::ip;    

//-----------------------------
// function
//-----------------------------
template<typename T, typename U> inline void forceSet(const T *value, const U &set_value){*((T*)value) = set_value;}

//-----------------------------
// Method
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace Sensor{

RFansDriver::RFansDriver(const std::string &ip_address, const int &status_port, const std::string &model)
{
    io_service io;
    this->status_socket = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status_port));

    RFansDeviceStatus status;
    for(int i=0; i<10 || !this->getDeviceInfo(&status); i++);
    if(status.mac_address == "") return;

    this->points_socket  = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status.points_port));
    this->command_socket = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status.command_port));
}

RFansDriver::~RFansDriver()
{

}

bool RFansDriver::scanStart(const int &hz)
{

}

bool RFansDriver::scanStop()
{

}

bool RFansDriver::getDeviceInfo(RFansDeviceStatus *status)
{
    char buf[32];
    boost::array<char, 512> recv_data;
    udp::endpoint endpoint;
    size_t len = status_socket->receive_from(boost::asio::buffer(recv_data), endpoint);
    if(len < 256) return false;
    status->header = (recv_data[0] & 0xff) << 24 | (recv_data[1] & 0xff) << 16 | (recv_data[2] & 0xff) << 8 | (recv_data[3] & 0xff);
    status->id     = (recv_data[4] & 0xff) << 24 | (recv_data[5] & 0xff) << 16 | (recv_data[6] & 0xff) << 8 | (recv_data[7] & 0xff);
    status->year   = 2000 + recv_data[8];
    status->month  = recv_data[9];
    status->day    = recv_data[10];
    status->hour   = recv_data[11];
    status->minute = recv_data[12];
    status->second = recv_data[13];
    std::snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x", recv_data[14]&0xff, recv_data[15]&0xff, recv_data[16]&0xff, recv_data[17]&0xff, recv_data[18]&0xff, recv_data[19]&0xff);
    status->mac_address  = buf;
    status->points_port   = (recv_data[20] & 0xff) << 8 | (recv_data[21] & 0xff);
    status->command_port = (recv_data[22] & 0xff) << 8 | (recv_data[23] & 0xff);
    status->motor_speed  = (recv_data[24] & 0xff) / 10.0;
    status->device_info  = (recv_data[25] & 0xff) << 24 | (recv_data[26] & 0xff) << 16 | (recv_data[27] & 0xff) << 8 | (recv_data[28] & 0xff);
    status->pps_encode   = (recv_data[29] & 0xff) << 8 | (recv_data[30] & 0xff);
    status->device_id    = (recv_data[31] & 0xff) << 8 | (recv_data[32] & 0xff);
    status->temperature  =((recv_data[33] & 0xff) << 8 | (recv_data[34] & 0xff)) / 100.0;
    status->check_sum    = (recv_data[35] & 0xff) << 24 | (recv_data[36] & 0xff) << 16 | (recv_data[37] & 0xff) << 8 | (recv_data[38] & 0xff);
    return true;
}

bool RFansDriver::getPoints(MiYALAB::Sensor::PolarCloud *polars)
{
    std::vector<RFansPointsPacket> packets;
    while(1){
        boost::array<char, 2048> recv_data;
        udp::endpoint endpoint;
        size_t len = points_socket->receive_from(boost::asio::buffer(recv_data), endpoint);
        if(len < 1206) continue;

        RFansPointsPacket packet;
        packet.groups.resize(12);
        for(int i=0; i<12; i++){
            auto *group = &recv_data[i*100];
            packet.groups[i].flag  = (group[0] & 0xff) << 8 | (group[1] & 0xff);
            packet.groups[i].angle =((group[3] & 0xff) << 8 | (group[2] & 0xff)) / 100.0;
            packet.groups[i].ranges.resize(32);
            packet.groups[i].intensity.resize(32);
            for(int j=0; j<32; j++){
                auto *point = &group[3*j+4];
                packet.groups[i].ranges[j]    =((point[1] & 0xff) << 8 | (point[0] & 0xff)) * 0.004;
                packet.groups[i].intensity[j] = (point[3] & 0xff) / 255.0;
            }
        }
        packet.timestamp = (recv_data[1200] & 0xff) | (recv_data[1201] & 0xff) << 8 | (recv_data[1202] & 0xff) << 16 | (recv_data[1203] & 0xff) << 24;
        packet.factory = (recv_data[1204] && 0xff) << 8 | (recv_data[1205] & 0xff);
        packets.emplace_back(packet);
    }

    for(const auto &packet: packets){
        for(const auto &group: packet.groups){
            MiYALAB::Mathematics::Polar32 polar;
            for(int i=0; i<32; i++){
                polar.range = group.ranges[i];
            }
        }
    }

    return true;
}
    
}
}

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------