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

#ifndef __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_DRIVER_HPP__
#define __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_DRIVER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <string>
#include <memory>

// Boost
#include <boost/asio.hpp>

// MiYALAB
#include "packet.hpp"
#include "device_status.hpp"

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Class
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace Sensor{

/**
 * @brief Component Definition
 * 
 */
class RFansDriver{
public:
    RFansDriver(const std::string &ip_address, const int &status_port, const std::string &model);
    virtual ~RFansDriver();
    bool scanStart(const int &hz);
    bool scanStop();
    bool getDeviceInfo(RFansDeviceStatus *status);
    bool getPoints();
private:
    std::shared_ptr<boost::asio::ip::udp::socket> status_socket;
    std::shared_ptr<boost::asio::ip::udp::socket> points_socket;
    std::shared_ptr<boost::asio::ip::udp::socket> command_socket;
};
}
}

#endif // __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_DRIVER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------