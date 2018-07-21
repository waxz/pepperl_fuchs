// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
// * Neither the name of Pepperl+Fuchs nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <pepperl_fuchs_r2000/r2000_driver.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"laser_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::cout << "Hello world!" << std::endl;
    std::string scanner_ip("192.168.7.10");
    pepperl_fuchs::R2000Driver driver;

    for( int i=0; i<2; i++ )
    {
        std::cout << "Connecting to scanner at " << scanner_ip << " ... ";
        if (driver.connect(scanner_ip, 80))
            std::cout << "OK" << std::endl;
        else
        {
            std::cout << "FAILED!" << std::endl;
            std::cerr << "Connection to scanner at " << scanner_ip << " failed!" << std::endl;
            return 1;
        }

        driver.setScanFrequency(35);
        driver.setSamplesPerScan(3600);
        auto params = driver.getParameters();
        std::cout << "Current scanner settings:" << std::endl;
        std::cout << "============================================================" << std::endl;
        for (const auto& p : params)
            std::cout << p.first << " : " << p.second << std::endl;
        std::cout << "============================================================" << std::endl;


        // Start capturing scanner data
        //-------------------------------------------------------------------------
        std::cout << "Starting capturing: ";
        if (driver.startCapturingUDP())
            std::cout << "OK" << std::endl;
        else
        {
            std::cout << "FAILED!" << std::endl;
            return 1;
        }
        driver.getOutputParameterList();

        for (int s = 0; s < 5; s++)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            int scans_captured = 0;
            int scans_available = driver.getFullScansAvailable();
            for (int i = 0; i < scans_available; i++)
            {
                auto scandata = driver.getScan();
                scans_captured++;
            }
            std::cout << "Received " << scans_captured << " from scanner" << std::endl;
        }

        std::cout << "Trying to stop capture" << std::endl;


    }

    std::cout << "Goodbye world!" << std::endl;

    std::cout << "Get full data!" << std::endl;

    if( !driver.isCapturing() )
    {
        std::cout << "ERROR: Laser range finder disconnected. Trying to reconnect..." << std::endl;

    }
    auto scandata = driver.getFullScan();
    if( scandata.amplitude_data.empty() || scandata.distance_data.empty() )
        return 0 ;

    sensor_msgs::LaserScan scanmsg;
    scanmsg.header.frame_id = "laser";
    scanmsg.header.stamp = ros::Time::now();

    scanmsg.angle_min = -M_PI;
    scanmsg.angle_max = +M_PI;
    scanmsg.angle_increment = 2*M_PI/float(scandata.distance_data.size());
    scanmsg.time_increment = 1/35.0f/float(scandata.distance_data.size());

    scanmsg.scan_time = 1/std::atof(driver.getParametersCached().at("scan_frequency").c_str());
    scanmsg.range_min = std::atof(driver.getParametersCached().at("radial_range_min").c_str());
    scanmsg.range_max = std::atof(driver.getParametersCached().at("radial_range_max").c_str());

    scanmsg.ranges.resize(scandata.distance_data.size());
    scanmsg.intensities.resize(scandata.amplitude_data.size());
    for( std::size_t i=0; i<scandata.distance_data.size(); i++ )
    {
        scanmsg.ranges[i] = float(scandata.distance_data[i])/1000.0f;
        scanmsg.intensities[i] = scandata.amplitude_data[i];
    }


    std::cout << "Stopping capture: " << driver.stopCapturing() << std::endl;

    return 0;
}
