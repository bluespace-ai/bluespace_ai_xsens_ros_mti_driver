// BSD 3-Clause License
//
// Copyright (c) 2021, BlueSpace.ai, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef TIMEREFERENCEPUBLISHER_H
#define TIMEREFERENCEPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/msg/time_reference.hpp>
#include <iomanip>      // std::get_time
#include <ctime>        // strcut std::tm

struct TimeReferencePublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub;
    int timeZoneOffset;

    TimeReferencePublisher(rclcpp::Node &node)
    {
        int pub_queue_size = 5;
        node.get_parameter("publisher_queue_size", pub_queue_size);
        timeZoneOffset = 0;
        node.get_parameter("time_zone_offset", timeZoneOffset);
        pub = node.create_publisher<sensor_msgs::msg::TimeReference>("/imu/time_ref", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsUtcTime())
        {
            const uint32_t SAMPLE_TIME_FINE_HZ = 10000UL;
            const uint32_t ONE_GHZ = 1000000000UL;
            uint32_t sec, nsec;
            sensor_msgs::msg::TimeReference msg;

            auto packetTime = packet.utcTime();
            // convert time struct to utc float time
            std::ostringstream date;
	        date << packetTime.m_year << 
        	"-" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_month) << 
        	"-" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_day) <<
        	"T" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_hour + timeZoneOffset) << 
        	":" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_minute) <<
        	":" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_second) << "Z";

            std::istringstream ss(date.str());
            std::tm t{};
            ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
            if (ss.fail()) {
                throw std::runtime_error{"failed to parse time string"};
            }   
            std::time_t time_stamp = mktime(&t);
            
            sec = int(time_stamp);
            nsec = packetTime.m_nano;

            rclcpp::Time sample_time(sec, nsec);

            msg.header.stamp = timestamp;
            // msg.header.frame_id = unused
            msg.time_ref = sample_time;
            // msg.source = optional

            pub->publish(msg);
        }
    }
};

#endif
