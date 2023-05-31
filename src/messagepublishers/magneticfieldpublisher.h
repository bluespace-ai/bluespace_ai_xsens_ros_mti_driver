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

//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
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

#ifndef MAGNETICFIELDPUBLISHER_H
#define MAGNETICFIELDPUBLISHER_H

#include "packetcallback.h"
#include "publisherhelperfunctions.h"
#include <sensor_msgs/msg/magnetic_field.hpp>

struct MagneticFieldPublisher : public PacketCallback, PublisherHelperFunctions
{
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    double magnetic_field_variance[3];

    MagneticFieldPublisher(rclcpp::Node &node)
    {
        std::vector<double> variance = {0, 0, 0};
        node.declare_parameter("magnetic_field_stddev", variance);

        int pub_queue_size = 5;
        node.get_parameter("publisher_queue_size", pub_queue_size);
        pub = node.create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", pub_queue_size);
        node.get_parameter("frame_id", frame_id);
        variance_from_stddev_param("magnetic_field_stddev", magnetic_field_variance, node);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsCalibratedMagneticField())
        {
            // Problem: Sensor gives normalized magnetic field vector with unknown units
            sensor_msgs::msg::MagneticField msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsVector mag = packet.calibratedMagneticField();

            msg.magnetic_field.x = mag[0];
            msg.magnetic_field.y = mag[1];
            msg.magnetic_field.z = mag[2];

            msg.magnetic_field_covariance[0] = magnetic_field_variance[0];
            msg.magnetic_field_covariance[4] = magnetic_field_variance[1];
            msg.magnetic_field_covariance[8] = magnetic_field_variance[2];

            pub->publish(msg);
        }
    }
};

#endif
