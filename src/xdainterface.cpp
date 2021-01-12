
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

#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"

// #define XS_DEFAULT_BAUDRATE (115200)

XdaInterface::XdaInterface()
	: Node("xsens_driver", rclcpp::NodeOptions())
	, m_device(nullptr)
	, m_xdaCallback(*this)
{
	declareCommonParameters();
	RCLCPP_INFO(get_logger(), "Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)
		{
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

void XdaInterface::registerPublishers()
{
	bool should_publish;
	rclcpp::Node& node = *this;

	if (get_parameter("pub_imu", should_publish) && should_publish)
	{
		registerCallback(new ImuPublisher(node));
	}
	if (get_parameter("pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(new OrientationPublisher(node));
	}
	if (get_parameter("pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(node));
	}
	if (get_parameter("pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(node));
	}
	if (get_parameter("pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(node));
	}
	if (get_parameter("pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(node));
	}
	if (get_parameter("pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(node));
	}
	if (get_parameter("pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(node));
	}
	if (get_parameter("pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(node));
	}
	if (get_parameter("pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(node));
	}
	if (get_parameter("pub_gnss", should_publish) && should_publish)
	{
		registerCallback(new GnssPublisher(node));
	}
	if (get_parameter("pub_twist", should_publish) && should_publish)
	{
		registerCallback(new TwistPublisher(node));
	}
	if (get_parameter("pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(new FreeAccelerationPublisher(node));
	}
	if (get_parameter("pub_transform", should_publish) && should_publish)
	{
		registerCallback(new TransformPublisher(node));
	}
}

bool XdaInterface::connectDevice()
{
	XsPortInfo mtPort;

	// TODO: Port this for allow_undeclared_parameters
	// if (ros::param::has("~port"))
	// {
	// 	std::string port_name;
	// 	int baudrate = XS_DEFAULT_BAUDRATE;

	// 	get_parameter("port", port_name);
	// 	get_parameter("baudrate", baudrate);

	// 	mtPort = XsPortInfo(port_name, XsBaud_numericToRate(baudrate));
	// }
	// else
	{
		RCLCPP_INFO(get_logger(), "Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts();

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				mtPort = portInfo;
				break;
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found");

	// TODO: Port for allow_undeclared_parameters 
	// std::string deviceId;
	// if (ros::param::get("~device_id", deviceId))
	// {
	// 	if (mtPort.deviceId().toString().c_str() != deviceId)
	// 		return handleError(std::string("Device with ID: %s not found") + deviceId);
	// }

	RCLCPP_INFO(get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), mtPort.baudrate());

	RCLCPP_INFO(get_logger(), "Opening port...");
	if (!m_control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	RCLCPP_INFO(get_logger(), "Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	// TODO: Port this for allow_undeclared_parameters
	// std::string log_file;
	// if (ros::param::get("~log_file", log_file))
	// {
	// 	if (m_device->createLogFile(log_file) != XRV_OK)
	// 		return handleError(std::string("Failed to create a log file! (%s)") + log_file);
	// 	else
	// 		RCLCPP_INFO(get_logger(), "Created a log file: %s", log_file.c_str());

	// 	if (!m_device->startRecording())
	// 		return handleError("Could not start recording");
	// }

	return true;
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
	RCLCPP_ERROR(get_logger(), "%s", error.c_str());
	return false;
}

void XdaInterface::declareCommonParameters()
{
	// Declare ROS parameters common to all the publishers
	std::string frame_id = DEFAULT_FRAME_ID;
	declare_parameter("frame_id", frame_id);

	int pub_queue_size = 5;
	declare_parameter("publisher_queue_size", pub_queue_size);

	bool should_publish = true;
	declare_parameter("pub_imu", should_publish);
	declare_parameter("pub_quaternion", should_publish);
	declare_parameter("pub_acceleration", should_publish);
	declare_parameter("pub_angular_velocity", should_publish);
	declare_parameter("pub_mag", should_publish);
	declare_parameter("pub_dq", should_publish);
	declare_parameter("pub_dv", should_publish);
	declare_parameter("pub_sampletime", should_publish);
	declare_parameter("pub_temperature", should_publish);
	declare_parameter("pub_pressure", should_publish);
	declare_parameter("pub_gnss", should_publish);
	declare_parameter("pub_twist", should_publish);
	declare_parameter("pub_free_acceleration", should_publish);
	declare_parameter("pub_transform", should_publish);
}