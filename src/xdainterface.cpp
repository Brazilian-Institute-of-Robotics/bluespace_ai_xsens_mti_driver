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

#include <chrono>
#include <thread>
#include <mutex>

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
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"

#define BIT(k, n) ((n >> k) & 1)

XdaInterface::XdaInterface(const std::string &node_name, const rclcpp::NodeOptions &options)
	: Node(node_name, options)
	, m_device(nullptr)
	, m_xdaCallback(*this)
{
	declareCommonParameters();
	RCLCPP_INFO(get_logger(), "Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);

	auto handle_goal = [&](
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<
			const bluespace_ai_xsens_mti_driver::action::SetNoRotation::Goal>
			goal) -> rclcpp_action::GoalResponse
		{
			(void)uuid;
			(void)goal;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		};

	auto handle_cancel = [&](const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<
		bluespace_ai_xsens_mti_driver::action::SetNoRotation>>
		goal_handle) -> rclcpp_action::CancelResponse
		{
			(void)goal_handle;
			return rclcpp_action::CancelResponse::REJECT;
		};

	auto handle_accepted = [&](const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<
		bluespace_ai_xsens_mti_driver::action::SetNoRotation>>
		goal_handle) {
			
        std::thread{
			std::bind(
				&XdaInterface::executeMgbe, this, std::placeholders::_1),
            goal_handle}.detach();
		};

    mgbe_server = rclcpp_action::create_server<bluespace_ai_xsens_mti_driver::action::SetNoRotation>(
		this,
		"set_no_rotation",
		handle_goal,
		handle_cancel,
		handle_accepted
	);
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
		if(rosPacket.second.containsStatus()){
			const std::lock_guard<std::mutex> lock(mutex);
			status_word = rosPacket.second.status();
		}

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
	bool checkDeviceID = false;
	std::string deviceId = "";

	// Check if scanning is enabled
	bool scan_for_devices = false;
	get_parameter("scan_for_devices", scan_for_devices);

	if (!scan_for_devices){
		// Read baudrate parameter
		int baudrateParam = 0;
		get_parameter("baudrate", baudrateParam);
		RCLCPP_INFO(get_logger(), "Found baudrate parameter: %d", baudrateParam);
		baudrate = XsBaud::numericToRate(baudrateParam);

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
		if (!XsScanner::scanPort(mtPort, baudrate))
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
	int port_counter = 0;
	while (port_counter < 10) {
		if (m_control->openPort(mtPort)) {
			break;
		}

		port_counter++;
		RCLCPP_INFO(get_logger(), "Failed to open port %s %d time...", mtPort.portName().toStdString().c_str(), port_counter);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	if (port_counter >= 10)
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

	RCLCPP_INFO(get_logger(), "Measuring ...");
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	// Perform a Manual Gyro Bias Estimation
	std::this_thread::sleep_for(std::chrono::seconds(1));
	RCLCPP_INFO(get_logger(), "Performing Manual Gyro Bias Estimation for 10 seconds");
	if (!perform_mgbe(10))
		return handleError("Could not finish Manual Gyro Bias Estimation");
	RCLCPP_INFO(get_logger(), "Finished Manual Gyro Bias Estimation");

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

	std::string reference_frame_id = DEFAULT_REFERENCE_FRAME_ID;
	declare_parameter("reference_frame_id", reference_frame_id);

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

	declare_parameter("enable_logging", false);
	declare_parameter("log_file", "log.mtb");
}

bool XdaInterface::perform_mgbe(uint16_t duration) {
	if (!m_device->setNoRotation(duration))
		return handleError("MGBE Command not received!");

	int counter = 0;
	auto start_t = std::chrono::system_clock::now();
	while (counter < 10) {
		if(std::chrono::system_clock::now() - start_t > std::chrono::seconds(1)){
			counter++;
			RCLCPP_INFO(get_logger(), "Calibrating ... %d/10", counter);
			start_t = std::chrono::system_clock::now();
		}
	}
	return true;
}

void XdaInterface::executeMgbe(
	const std::shared_ptr<rclcpp_action::ServerGoalHandle<
		bluespace_ai_xsens_mti_driver::action::SetNoRotation>>
		goal_handle) 
{
	auto result = std::make_shared<
		bluespace_ai_xsens_mti_driver::action::SetNoRotation::Result>();

	const auto goal = goal_handle->get_goal();

	if (m_device->setNoRotation(goal->seconds)) {
		auto check_mgbe_running = [&]() -> bool {
			const std::lock_guard<std::mutex> lock(mutex);
			return BIT(3, status_word) & BIT(4, status_word);
		};

		auto start = std::chrono::system_clock::now();
		do {
			std::this_thread::sleep_for(std::chrono::seconds(2));
			RCLCPP_INFO(get_logger(), "Calibrating ... %d/%d", 
						static_cast<int>((std::chrono::system_clock::now() - start).count() * 1e-9), 
						goal->seconds);
		} while (check_mgbe_running());
		
		const std::lock_guard<std::mutex> lock(mutex);
		
		if (BIT(3, status_word) ^ BIT(4, status_word)){
			RCLCPP_ERROR(get_logger(), "Motion detected, MGBE failed!");
			result->result = false;
			goal_handle->abort(result);

		} else {
			RCLCPP_INFO(get_logger(), "Successful MGBE");
			result->result = true;
			goal_handle->succeed(result);
		}

	} else {
		RCLCPP_ERROR(get_logger(), "SetNoRotation msg not received");
		result->result = false;
		goal_handle->abort(result);
	}
}
