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

#include "xdainterface.h"
#include "xdautils.h"

#include <vector>

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsfilterprofilearray.h>

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
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"

XdaInterface::XdaInterface(const std::string &node_name, const rclcpp::NodeOptions &options)
	: Node(node_name, options)
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
	RCLCPP_INFO(get_logger(), "Cleaning up ...");
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
	if (get_parameter("pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(new PositionLLAPublisher(node));
	}
	if (get_parameter("pub_velocity", should_publish) && should_publish)
	{
		registerCallback(new VelocityPublisher(node));
	}
}

bool XdaInterface::connectDevice()
{
	XsPortInfo mtPort;
	XsBaudRate baudrate = XBR_Invalid;
	XsBaudRate config_baudrate = XBR_Invalid;
	bool checkDeviceID = false;
	std::string deviceId = "";

	// Check if scanning is enabled
	bool scan_for_devices = false;
	get_parameter("scan_for_devices", scan_for_devices);

	if (!scan_for_devices){
		// Read baudrate parameter(s)
		int baudrateParam = 0;
		get_parameter("baudrate", baudrateParam);
		RCLCPP_INFO(get_logger(), "Found baudrate parameter: %d", baudrateParam);
		baudrate = XsBaud::numericToRate(baudrateParam);

		int config_baudrateParam = 0;
		if (get_parameter("config_baudrate", config_baudrateParam))
		{
			RCLCPP_INFO(get_logger(), "Found config_baudrate parameter: %d", config_baudrateParam);
			config_baudrate = XsBaud::numericToRate(config_baudrateParam);
		}

		// Read device ID parameter if set
		get_parameter("device_id", deviceId);
		if (deviceId != "")
		{
			checkDeviceID = true;
			RCLCPP_INFO(get_logger(), "Found device ID parameter: %s.", deviceId.c_str());
		}

		// Read port parameter
		std::string portName;
		get_parameter("port", portName);
		RCLCPP_INFO(get_logger(), "Found port name parameter: %s", portName.c_str());
		mtPort = XsPortInfo(portName, baudrate);
		RCLCPP_INFO(get_logger(), "Scanning port %s ...", portName.c_str());
		if (!XsScanner::scanPort(mtPort, baudrate) && !XsScanner::scanPort(mtPort, config_baudrate))
			return handleError("No MTi device found. Verify port and baudrate.");
		if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
			return handleError("No MTi device found with matching device ID.");
	}
	else
	{
		RCLCPP_INFO(get_logger(), "Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found.");

	RCLCPP_INFO(get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	RCLCPP_INFO(get_logger(), "Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	RCLCPP_INFO(get_logger(), "Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	if (config_baudrate != XBR_Invalid)
	{
		const auto actualBaudrate = XsBaud::rateToNumeric(m_device->baudRate());
		if (m_device->baudRate() != config_baudrate)
		{
			RCLCPP_INFO(get_logger(), "Detected baudrate: %d, configured baudrate: %d", XsBaud::rateToNumeric(m_device->baudRate()), XsBaud::rateToNumeric(config_baudrate));
			RCLCPP_INFO(get_logger(), "Configuring baudrate to: %d", XsBaud::rateToNumeric(config_baudrate));
			if (!m_device->setSerialBaudRate(config_baudrate))
				return handleError("Could not configure baudate");
		}
	}

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::configureDevice()
{
	const auto profiles = m_device->availableOnboardFilterProfiles();
	RCLCPP_INFO(get_logger(), "Supported device profiles:");
	for (const auto& profile : profiles)
	{
		RCLCPP_INFO(get_logger(), " - %s", profile.label());
	}

	const auto current_profile = m_device->onboardFilterProfile();
	RCLCPP_INFO(get_logger(), "Profile in use: %s", current_profile.label());

	std::string selected_profile;
	if (get_parameter("onboard_filter_profile", selected_profile) && !selected_profile.empty())
	{
		RCLCPP_INFO(get_logger(), "Found filter profile parameter: %s", selected_profile.c_str());

		if (current_profile.label() == selected_profile)
		{
			RCLCPP_INFO(get_logger(), "Onboard filter profile already setup to correctly");
		}
		else
		{
			RCLCPP_INFO(get_logger(), "Selecting onboard filter profile: %s", selected_profile.c_str());
			if (!m_device->setOnboardFilterProfile(selected_profile))
			{
				return handleError("Could not set onboard filter profile");
			}
		}
	}

	const auto current_output_configuration = m_device->outputConfiguration();
	RCLCPP_INFO(get_logger(), "Currently configured output configuration:");
	for (const auto& cfg : current_output_configuration) {
		const auto output_name = get_xs_data_identifier_name(cfg.m_dataIdentifier);
		RCLCPP_INFO(get_logger(), " - %s = %d", output_name.c_str(), cfg.m_frequency);
	}

	std::vector<std::string> output_configuration;
	if (get_parameter("output_configuration", output_configuration) && !output_configuration.empty())
	{
		RCLCPP_INFO(get_logger(), "Found output configuration parameter(s):");

		XsOutputConfigurationArray newConfigArray;
		for (const auto& cfg : output_configuration)
		{
			std::string output_name;
			int output_frequency;
			if (!parseConfigLine(cfg, output_name, output_frequency))
			{
				return handleError("Could not parse line" + cfg);
			}
			RCLCPP_INFO(get_logger(), " - %s = %d", output_name.c_str(), output_frequency);

			XsDataIdentifier data_identifier;
			if (!get_xs_data_identifier_by_name(output_name, data_identifier))
			{
				return handleError("Invalid data identifier: " + output_name);
			}

			newConfigArray.push_back(XsOutputConfiguration(data_identifier, output_frequency));
		}

		if (newConfigArray == current_output_configuration)
		{
			RCLCPP_INFO(get_logger(), "Output configuration already configured correctly");
		}
		else
		{
			RCLCPP_INFO(get_logger(), "Setting output configuration");
			if (!m_device->setOutputConfiguration(newConfigArray))
			{
				return handleError("Could not set output configuration");
			}
		}
	}

	auto configureAlignmentQuat = [&](const std::string& name)
	{
		const auto frame = (name == "sensor") ? XAF_Sensor : XAF_Local;
		const auto parameterName = (frame == XAF_Sensor) ? "alignment_sensor_quat" : "alignment_local_quat";

		std::vector<XsReal> alignment_quat;
		if (get_parameter(parameterName, alignment_quat) && !alignment_quat.empty())
		{
			RCLCPP_INFO(get_logger(), "Configuration alignment rotation for %s", parameterName);
			const auto currentAlignmentQuat = m_device->alignmentRotationQuaternion(frame);
			RCLCPP_INFO(get_logger(), " - current alignment quaternion for %s (%d): [%f %f %f %f]",
				parameterName, frame, currentAlignmentQuat.w(), currentAlignmentQuat.x(),
				currentAlignmentQuat.y(), currentAlignmentQuat.z());

			const auto paramAlignmentQuat = XsQuaternion{alignment_quat[0], alignment_quat[1], alignment_quat[2], alignment_quat[3]};
			if (!paramAlignmentQuat.isEqual(currentAlignmentQuat, 0.01))
			{
				RCLCPP_INFO(get_logger(), " - desired alignment quaternion for %s (%d): [%f %f %f %f]", parameterName, frame,
					paramAlignmentQuat.w(), paramAlignmentQuat.x(), paramAlignmentQuat.y(), paramAlignmentQuat.z());
				if (!m_device->setAlignmentRotationQuaternion(frame, paramAlignmentQuat))
					return handleError("Could not configure alignment quaternion");
			}
			else
			{
				RCLCPP_INFO(get_logger(), " - actual and desired are near each other, no action taken");
			}
		}

		return true;
	};

	if (!configureAlignmentQuat("local"))
		return handleError("Could not set the local rotation matrix");
	if (!configureAlignmentQuat("sensor"))
		return handleError("Could not set the sensor rotation matrix");

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	if (!configureDevice())
		return handleError("Could not not apply the custom device configuration");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	RCLCPP_INFO(get_logger(), "Measuring ...");
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	bool enable_logging = false;
	get_parameter("enable_logging", enable_logging);

	if (enable_logging)
	{
		std::string logFile;
		if (get_parameter("log_file", logFile))
		{
			if (m_device->createLogFile(logFile) != XRV_OK)
				return handleError("Failed to create a log file! (" + logFile + ")");
			else
				RCLCPP_INFO(get_logger(), "Created a log file: %s", logFile.c_str());

			RCLCPP_INFO(get_logger(), "Recording to %s ...", logFile.c_str());
			if (!m_device->startRecording())
				return handleError("Could not start recording");
		}
	}

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
	declare_parameter("pub_positionLLA", should_publish);
	declare_parameter("pub_velocity", should_publish);

	declare_parameter("scan_for_devices", true);
	declare_parameter("device_id", "");
	declare_parameter("port", "");
	declare_parameter("baudrate", XsBaud::rateToNumeric(XBR_Invalid));

	declare_parameter<std::string>("onboard_filter_profile", "");
	declare_parameter<std::vector<std::string>>("output_configuration", std::vector<std::string>());
	declare_parameter("config_baudrate", XsBaud::rateToNumeric(XBR_Invalid));
	declare_parameter<std::vector<XsReal>>("alignment_local_quat", {1., 0., 0., 0.});
	declare_parameter<std::vector<XsReal>>("alignment_sensor_quat", {1., 0., 0., 0.});

	declare_parameter("enable_logging", false);
	declare_parameter("log_file", "log.mtb");
}
