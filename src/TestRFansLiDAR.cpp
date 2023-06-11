#include <iostream>

#include <opencv2/opencv.hpp>

#include <MiYALAB/Sensor/PointCloud/PointCloudPolar.hpp>
#include <MiYALAB/Sensor/RFansLiDAR/driver.hpp>

using namespace MiYALAB::Sensor;

int main(int argc, char **argv)
{
    RFansDriver rfans("192.168.0.3", 2030, "R-Fans-16");
    rfans.scanStart(20);
    RFansDeviceStatus status;
    rfans.getDeviceInfo(&status);

    std::printf("RFans Driver create by MiYALAB");
    std::printf("mac addr: %s", status.mac_address.c_str());
    std::printf("header  : %08x", status.header);
    std::printf("time    : %04d/%02d/%02d %02d:%02d:%02d", status.year, status.month, status.day, status.hour, status.minute, status.second);
    std::printf("rps     : %lf", status.motor_speed);
    std::printf("temp    : %lf", status.temperature);

    while(1){
        PointCloudPolar polars;
        if(!rfans.getPoints(&polars)){
            std::cerr << "failure lidar device scan" << std::endl;
        }

        constexpr double res = 0.01;
        cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0,0,0));
        for(int i=0, size=polars.polars.size(); i<size; i++){
            double x = polars.polars[i].range * std::cos(polars.polars[i].phi) * std::cos(polars.polars[i].theta);
            double y = polars.polars[i].range * std::cos(polars.polars[i].phi) * std::sin(polars.polars[i].theta);
            double z = polars.polars[i].range * std::sin(polars.polars[i].phi);

            int px = img.cols/2 - y/res;
            int py = img.rows/2 - x/res;
            if(0<=px && px<img.cols && 0<=py && py<img.rows) img.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
        }
    }
}