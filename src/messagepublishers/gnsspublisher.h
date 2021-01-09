
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#ifndef GNSSPUBLISHER_H
#define GNSSPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#define FIX_TYPE_2D_FIX (2)
#define FIX_TYPE_3D_FIX (3)
#define FIX_TYPE_GNSS_AND_DEAD_RECKONING (4)

struct GnssPublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    GnssPublisher(rclcpp::Node &node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/gnss", pub_queue_size);
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsRawGnssPvtData())
        {
            sensor_msgs::msg::NavSatFix msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsRawGnssPvtData gnss = packet.rawGnssPvtData();

            msg.latitude = (double)gnss.m_lat * 1e-7;
            msg.longitude = (double)gnss.m_lon * 1e-7;
            msg.altitude = (double)gnss.m_height * 1e-3;
            // Position covariance [m^2], ENU
            double sh = ((double)gnss.m_hAcc * 1e-3);
            double sv = ((double)gnss.m_vAcc * 1e-3);
            msg.position_covariance = {sh * sh, 0, 0, 0, sh * sh, 0, 0, 0, sv * sv};
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            switch (gnss.m_fixType)
            {
            case FIX_TYPE_2D_FIX: // fall through
            case FIX_TYPE_3D_FIX: // fall through
            case FIX_TYPE_GNSS_AND_DEAD_RECKONING:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                break;
            default:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }
            msg.status.service = 0; // unknown

            pub->publish(msg);
        }
    }
};

#endif
