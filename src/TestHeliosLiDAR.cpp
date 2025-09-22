#include <iostream>
#include <chrono>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <MiYALAB/Device/LiDAR/RoboSense/driver.hpp>

using namespace MiYALAB::Device::LiDAR;

int main(int argc, char **argv)
{
    std::printf("connect...\n");
    RoboSense helios("192.168.1.200", 7788, 6699);
    RoboSense::LiDARInfo status;
    helios.getDeviceInfo(&status);

    std::printf("RS-Helios Driver create by MiYALAB\n");
    std::printf("mac addr: %s\n", status.mac_address.c_str());
    std::printf("data port: %d\n", status.data_port);
    std::printf("status port: %d\n", status.status_port);
    // std::printf("header  : %08x\n", status.header);
    // std::printf("time    : %04d/%02d/%02d %02d:%02d:%02d\n", status.year, status.month, status.day, status.hour, status.minute, status.second);
    std::printf("rps     : %d\n", status.motor_speed);
    // std::printf("temp    : %lf\n", status.temperature);

    auto start = std::chrono::high_resolution_clock::now();
    helios.startScan(20);
    while(cv::waitKey(1) != 'q'){
        // std::cout << "scan start..." << std::endl;
        auto stop = std::chrono::high_resolution_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(stop-start).count() / 1e6 << "ms " << std::endl;
        start = stop;

        MiYALAB::Sensor::PointCloud points;
        if(!helios.getPoints(&points)){
            std::cerr << "failure lidar device scan" << std::endl;
        }
        // std::cout << "scan finish" << std::endl;

        constexpr double res = 0.05;
        cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(200,200,200));
        for(int i=0, size=points.points.size(); i<size; i++){

            double x = points.points[i].x;
            double y = points.points[i].y;
            double z = points.points[i].z;

            int px = img.cols/2 - y/res;
            int py = img.rows/2 - x/res;

            if(0<=px && px<img.cols && 0<=py && py<img.rows){
                img.at<cv::Vec3b>(py,px) = cv::Vec3b(0,0,0);
            }
        }

        cv::putText(img, std::to_string(points.channels[2].values[0] / M_PI * 180), cv::Point(10,10), 1, 1.0, cv::Scalar(0,0,0));
        cv::line(img, 
            cv::Point(500, 500),
            cv::Point(500.0 - 500.0 * std::sin(points.channels[2].values[0]), 500.0 - 500.0 * std::cos(points.channels[2].values[0])),
            cv::Scalar(255,0,0),
            1
        );
        cv::circle(img, cv::Point(500,500), 100, cv::Scalar(0,0,255), 1);
        cv::circle(img, cv::Point(500,500), 250, cv::Scalar(0,0,255), 1);
        cv::circle(img, cv::Point(500,500), 500, cv::Scalar(0,0,255), 1);
        cv::line(img, cv::Point(0, 500), cv::Point(1000, 500), cv::Scalar(0,0,255), 1);
        cv::line(img, cv::Point(500, 0), cv::Point(500, 1000), cv::Scalar(0,0,255), 1);
        
        cv::imshow("img", img);
    }
    cv::destroyAllWindows();
    // rfans.scanStop();
}