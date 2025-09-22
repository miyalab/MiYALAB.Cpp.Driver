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

#ifndef __MIYALAB_DRIVER_DEVICE_LIDAR_ROBOSENSE_DRIVER_HPP__
#define __MIYALAB_DRIVER_DEVICE_LIDAR_ROBOSENSE_DRIVER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <string>
#include <memory>

// Boost
#include <boost/asio.hpp>

// MiYALAB
#include <MiYALAB/Device/LiDAR/LiDAR.hpp>

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
namespace Device{
namespace LiDAR{
/**
 * @brief Component Definition
 * 
 */
class RoboSense{
public:
    RoboSense(const std::string &ip_address, const int &status_port, const int &data_port);
    virtual ~RoboSense();
    bool startScan();
    bool startScan(const int &hz) {return startScan();}
    bool stopScan();
    bool getPoints(MiYALAB::Sensor::PointCloud *points);
    
    struct LiDARInfo{
        uint64_t header;
        uint16_t motor_speed;
        std::string device_ip;
        std::string destination_ip;
        std::string mac_address;
        uint16_t status_port;
        uint16_t data_port;

        uint8_t return_mode;

        bool laser_status;

        std::vector<double> vertical_angle_correct;
        std::vector<double> horizontal_angle_correct;
        
        double top_board_temp;
        double bottom_board_temp;
    };
    bool getDeviceInfo(LiDARInfo *status);

private:
    int data_port = 0;
    int status_port = 0;
    bool is_running = false;

    std::shared_ptr<boost::asio::ip::udp::socket> status_socket = nullptr;
    std::shared_ptr<boost::asio::ip::udp::socket> data_socket = nullptr;
    
    uint8_t return_mode;
    std::vector<double> vertical_angle_correct;
    std::vector<double> horizontal_angle_correct;

    uint32_t seq = 0;
};
}
}
}

#endif // __MIYALAB_DRIVER_DEVICE_LIDAR_ROBOSENSE_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------