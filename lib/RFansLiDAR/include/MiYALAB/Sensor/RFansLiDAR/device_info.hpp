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

#ifndef __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_DEVICE_INFO_HPP__
#define __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_DEVICE_INFO_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <string>

// Boost

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Struct
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace Sensor{
struct RFansDeviceInfo{
    unsigned int header;
    unsigned short id;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    std::string mac_address;
    unsigned short point_port;
    unsigned short command_port;
    unsigned char motor_speed;
    unsigned int device_info;
};
}
}

#endif // __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_DEVICE_INFO_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------