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
#include <iostream>
#include <string>
#include <memory>
#include <future>
#include <ctime>

// Boost
#include <boost/array.hpp>
#include <boost/asio/ip/udp.hpp>

// MiYALAB
#include "MiYALAB/Device/LiDAR/RoboSense/driver.hpp"

//-----------------------------
// Namespace & using
//-----------------------------
using namespace boost::asio;
using namespace boost::asio::ip;
using MiYALAB::Sensor::PointCloud;

//-----------------------------
// Const value
//-----------------------------
constexpr double PI_2 = 2 * M_PI;
constexpr double TO_RAD = M_PI / 180;
constexpr char RS_LIDAR_MODEL_NAME[][13] = {
    "RS",
    "RS-LiDAR-16",
    "RS-LiDAR-32",
    "RS-Bpearl",
    "RS-Ruby",
    "RS-Ruby Lite",
    "RS-Helios"
};

//-----------------------------
// Method
//-----------------------------
namespace MiYALAB{
namespace Device{
namespace LiDAR{
RoboSense::RoboSense(const std::string &ip_address, const int &status_port, const int &data_port)
{
    LiDARInfo info;
    io_service io;
    this->status_socket = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status_port));
    this->data_socket   = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), data_port));
    this->getDeviceInfo(&info);
}

RoboSense::~RoboSense()
{
    this->stopScan();
}

bool RoboSense::startScan()
{
    LiDARInfo info;
    this->getDeviceInfo(&info);

    this->return_mode = info.return_mode;
    this->vertical_angle_correct = info.vertical_angle_correct;
    this->horizontal_angle_correct = info.horizontal_angle_correct;

    io_service io;
    this->is_running  = true;
    return true;
}

bool RoboSense::stopScan()
{    
    this->data_socket = nullptr;
    this->is_running = false;
    return true;
}

bool RoboSense::getDeviceInfo(LiDARInfo *status)
{
    // デバイス情報ポート接続
    boost::array<uint8_t, 2496> recv_data;
    udp::endpoint endpoint;
    size_t len = this->status_socket->receive_from(boost::asio::buffer(recv_data), endpoint);
    if(len < 1048) return false;
    
    // ヘッダー
    status->header = (uint64_t)recv_data[0] << 56 | (uint64_t)recv_data[1] << 48 | (uint64_t)recv_data[2] << 40 | (uint64_t)recv_data[3] << 32
                    | recv_data[4] << 24 | recv_data[5] << 16 | recv_data[6] << 8 | recv_data[7];
    
    // モータ回転数
    status->motor_speed = recv_data[8] << 8 | recv_data[9];
    
    // Ethernet情報
    status->device_ip      = std::to_string(recv_data[10]) + ":" + std::to_string(recv_data[11]) + ":" + std::to_string(recv_data[12]) + ":" + std::to_string(recv_data[13]);
    status->destination_ip = std::to_string(recv_data[14]) + ":" + std::to_string(recv_data[15]) + ":" + std::to_string(recv_data[16]) + ":" + std::to_string(recv_data[17]);
    char buf[32];
    std::snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x", recv_data[18], recv_data[19], recv_data[20], recv_data[21], recv_data[22], recv_data[23]);
    status->mac_address = buf;
    status->status_port = recv_data[24] << 8 | recv_data[25];
    status->data_port   = recv_data[28] << 8 | recv_data[29];

    // リターンモード
    status->return_mode = recv_data[300];

    // 温度取得
    status->top_board_temp    = 503.975 * (recv_data[342] << 8 | recv_data[343]) / 4095 - 273.15;
    status->bottom_board_temp = 503.975 * (recv_data[350] << 8 | recv_data[351]) / 4095 - 273.15;

    // 垂直角度補正
    auto correct_ptr = &recv_data[468];
    for(int i=0; i<32; i++){
        auto channel_ptr = &correct_ptr[3*i];
        status->vertical_angle_correct.emplace_back((double)(channel_ptr[1] << 8 | channel_ptr[2]) * (channel_ptr[0] != 0 ? -1.0 : 1.0) * 0.01 * TO_RAD); 
    }

    // 水平補正角
    correct_ptr = &recv_data[564];
    for(int i=0; i<32; i++){
        auto channel_ptr = &correct_ptr[3*i];
        status->horizontal_angle_correct.emplace_back((double)(channel_ptr[1] << 8 | channel_ptr[2]) * (channel_ptr[0] != 0 ? -1.0 : 1.0) * 0.01 * TO_RAD); 
    }
    return true;
}

bool RoboSense::getPoints(PointCloud *points)
{   
    if(!is_running) return false;

    double angle_sum = 0;
    double pre_angle = 0;
    uint32_t cnt = 0;
    points->points.clear();
    points->channels.clear();
    points->channels.resize(5);
    points->channels[0].name = "range";
    points->channels[1].name = "indensity";
    points->channels[2].name = "h_angle";
    points->channels[3].name = "v_angle";
    points->channels[4].name = "delta_t";
    
    while(angle_sum < PI_2){
        // データ受信待ち
        boost::array<uint8_t, 2496> recv_data;
        udp::endpoint endpoint;
        size_t len = this->data_socket->receive_from(boost::asio::buffer(recv_data), endpoint);
        if(len < 1048) continue;
        
        // 計算用データ
        double range_resolution = (recv_data[17] != 0 ? 0.025 : 0.050);
        uint64_t seconds = (uint64_t)recv_data[20] << 40
                    | (uint64_t)recv_data[21] << 32
                    | (uint64_t)recv_data[22] << 24 
                    | (uint64_t)recv_data[23] << 16 
                    | (uint64_t)recv_data[24] << 8 
                    | (uint64_t)recv_data[25];
        uint32_t nanoseconds = (recv_data[26] << 24 | recv_data[27] << 16 | recv_data[28] << 8 | recv_data[29]) * 1000;
        auto data_ptr = &recv_data[42];
        
        // 初回受信
        if(cnt == 0){
            // ヘッダー生成
            points->header.frame_id = (std::string)RS_LIDAR_MODEL_NAME[recv_data[31]];
            points->header.stamp.seconds = seconds;
            points->header.stamp.nanoseconds = nanoseconds;
            points->header.seq = this->seq;

            // 取得角度累計計算用
            pre_angle = (double)(data_ptr[2] << 8 | data_ptr[3]) * 0.01 * TO_RAD - M_PI;
        }

        // 点群データ読み込み
        double horizontal_angle = 0;
        double delta_t = (double)(seconds - points->header.stamp.seconds) + (double)(nanoseconds - points->header.stamp.nanoseconds) / 1e9;
        for(int i=0; i<12; i++){
            auto block_ptr = &data_ptr[100*i];
            horizontal_angle = (double)(block_ptr[2] << 8 | block_ptr[3]) * 0.01 * TO_RAD - M_PI;

            // チャンネル
            for(int j=0; j<32; j++){
                // 距離, 反射率, 水平角, 垂直角, タイムスタンプからの経過時間
                auto channel_ptr = &block_ptr[3*j+4];
                points->channels[0].values.emplace_back((double)(channel_ptr[0] << 8 | channel_ptr[1]) * range_resolution);
                points->channels[1].values.emplace_back((double)channel_ptr[2] / 255.0);
                points->channels[2].values.emplace_back(-(horizontal_angle + this->horizontal_angle_correct[j]));
                points->channels[3].values.emplace_back(this->vertical_angle_correct[j]);
                points->channels[4].values.emplace_back(delta_t);
    
                // 点情報
                MiYALAB::Mathematics::Point point;
                points->points.emplace_back(
                    points->channels[0].values[cnt] * std::cos(points->channels[3].values[cnt]) * std::cos(points->channels[2].values[cnt]),
                    points->channels[0].values[cnt] * std::cos(points->channels[3].values[cnt]) * std::sin(points->channels[2].values[cnt]),
                    points->channels[0].values[cnt] * std::sin(points->channels[3].values[cnt])
                );
                cnt++;
            }
        }

        // 点群取得水平角度計算
        double delta_angle = horizontal_angle - pre_angle;
        if(delta_angle < -M_PI)     delta_angle += PI_2;
        else if(delta_angle > M_PI) delta_angle -= PI_2;
        pre_angle = horizontal_angle;
        angle_sum += std::abs(delta_angle);
    }

    // 水平角度の正規化
    for(int i=0; i<cnt; i++){
        if     (points->channels[2].values[i] < -M_PI) points->channels[2].values[i] += PI_2;
        else if(points->channels[2].values[i] >  M_PI) points->channels[2].values[i] -= PI_2;
    }
    
    this->seq++;
    return true;
}

}
}
}

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------