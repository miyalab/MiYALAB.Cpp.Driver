#include <iostream>
#include <chrono>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <MiYALAB/Sensor/PointCloud/PointCloudPolar.hpp>
#include <MiYALAB/Sensor/RFansLiDAR/driver.hpp>

using namespace MiYALAB::Sensor;

int main(int argc, char **argv)
{
    std::printf("connect...\n");
    RFansDriver rfans("192.168.0.3", 2030, "R-Fans-16");
    std::printf("ok\n");
    rfans.scanStart(20);
    RFansDeviceStatus status;
    rfans.getDeviceInfo(&status);

    std::printf("RFans Driver create by MiYALAB\n");
    std::printf("mac addr: %s\n", status.mac_address.c_str());
    std::printf("header  : %08x\n", status.header);
    std::printf("time    : %04d/%02d/%02d %02d:%02d:%02d\n", status.year, status.month, status.day, status.hour, status.minute, status.second);
    std::printf("rps     : %lf\n", status.motor_speed);
    std::printf("temp    : %lf\n", status.temperature);

    auto start = std::chrono::high_resolution_clock::now();
    while(cv::waitKey(1) != 'q'){
        // std::cout << "scan start..." << std::endl;
        auto stop = std::chrono::high_resolution_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(stop-start).count() / 1e6 << "ms " << std::endl;
        start = stop;
        PointCloudPolar polars;
        if(!rfans.getPoints(&polars)){
            std::cerr << "failure lidar device scan" << std::endl;
        }
        // std::cout << "scan finish" << std::endl;

        constexpr double res = 0.02;
        cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0,0,0));
        double max = -1e9;
        double min = 1e9;
        for(int i=0, size=polars.polars.size(); i<size; i++){
            max = std::max(max, polars.polars[i].theta);
            min = std::min(min, polars.polars[i].theta);
            // std::cout << "calc xyz..." << std::endl;
            double x = polars.polars[i].range * std::cos(polars.polars[i].phi) * std::cos(polars.polars[i].theta);
            double y = polars.polars[i].range * std::cos(polars.polars[i].phi) * std::sin(polars.polars[i].theta);
            double z = polars.polars[i].range * std::sin(polars.polars[i].phi);
            // std::cout << "calc finish" << std::endl;
            int px = img.cols/2 - y/res;
            int py = img.rows/2 - x/res;
            if(0<=px && px<img.cols && 0<=py && py<img.rows){
                // std::cout << "(x,y): (" << px << ", " << py << ")" << std::endl;
                img.at<cv::Vec3b>(py,px) = cv::Vec3b(255,255,255);
            }
            // std::cout << "draw point" << std::endl;
        }

        cv::imshow("img", img);
        // std::cout << "(" << min << ", " << max << ")" << std::endl;
    }
    cv::destroyAllWindows();
    rfans.scanStop();
}