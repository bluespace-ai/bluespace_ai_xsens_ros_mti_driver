
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
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

#include "mtdevice.h"
#include <xstypes/xsens_debugtools.h>
#include <xstypes/xssensorranges.h>
#include <xscommon/xsens_janitors.h>
#include <xstypes/xsquaternion.h>
#include <xstypes/xsmatrix.h>
#include <xstypes/xsmath.h>
#include <xstypes/xsdid.h> // for watermark checking
#include <xstypes/xssyncsettingarray.h>
#include <xstypes/xstime.h>
#include <xstypes/xssyncsetting.h>
#include <xstypes/xsdatapacket.h>
#include "communicator.h"
#include <xstypes/xsscrdata.h>
#include <xstypes/xscalibrateddata.h>
#include <xstypes/xssdidata.h>
#include "xserrormode.h"
#include "xsresetmethod.h"
#include "scenariomatchpred.h"
#include <algorithm>
#include "mtsyncsettings.h"
#include <xstypes/xsoutputconfigurationarray.h>
#include <algorithm>

#include <xstypes/xsstatusflag.h>

using namespace xsens;

/*! \brief 'Less-than' implementation for 'XsFilterProfile'
	Sorts on the 'type'
*/
bool MtDevice::CompareXsFilterProfile::operator() (XsFilterProfile const & left, XsFilterProfile const & right) const
{
#if 0 // Compare by string
	std::ostringstream leftStream, rightStream;
	leftStream << left.type() << '.' << left.version();
	rightStream << right.type() << '.' << right.version();
	return leftStream.str() < rightStream.str();
#endif
	return left.type() < right.type();
}

/*! \brief Constructs a standalone MtDevice based on \a comm
*/
MtDevice::MtDevice(Communicator* comm)
	: XsDeviceEx(comm)
{
}

/*! \brief Constructs a standalone MtDevice based on \a master and \a childDeviceId
*/
MtDevice::MtDevice(MtContainer * master, const XsDeviceId &childDeviceId)
	: XsDeviceEx(master, childDeviceId)
{
}

/*! \brief Destroys the MtDevice
*/
MtDevice::~MtDevice()
{
	JLTRACEG("entry");
	XSEXITLOGN(gJournal);
}
/*! \brief Checks for the sanity of a message
	\param msg A message to check
	\returns True if successful
*/
bool MtDevice::messageLooksSane(const XsMessage &msg) const
{
	return msg.getBusId() == 1 || XsDevice::messageLooksSane(msg);
}

/*! \brief Initialize the Mt device using the supplied filter profiles
	\param filterProfiles The filter profiles
	\returns True if successful
*/
bool MtDevice::initialize(const xsens::SettingsFile& filterProfiles)
{
	if (!XsDeviceEx::initialize(filterProfiles))
		return false;

	// we must create the data caches first so they are available even if the rest of the init fails
	// when reading from file almost all init can fail but we can still read data into the caches
	if (!readDeviceConfiguration())
	{
		setInitialized(false);
		return false;
	}

	fetchAvailableHardwareScenarios();
	updateScenarios();

	return true;
}

/*! \brief Updates the scenarios
	\returns True if successful
*/
bool MtDevice::updateScenarios()
{
	const XsMtDeviceConfiguration& info = deviceConfiguration().deviceInfo(deviceId());
	if (info.m_filterProfile != 0)
	{
		m_hardwareFilterProfile = XsFilterProfile(info.m_filterProfile & 0xFF
			,info.m_filterProfile >> 8
			,m_hardwareFilterProfile.label()
			,info.m_filterType
			,info.m_filterMajor
			,info.m_filterMinor);
	}

	for (std::vector<XsFilterProfile>::const_iterator i = m_hardwareFilterProfiles.begin(); i != m_hardwareFilterProfiles.end(); ++i)
	{
		if (i->type() == m_hardwareFilterProfile.type())
		{
			m_hardwareFilterProfile.setLabel(i->label());
			m_hardwareFilterProfile.setVersion(i->version());
			break;
		}
	}

	return true;
}

/*! \returns True if this is a motion tracker
*/
bool MtDevice::isMotionTracker() const
{
	return true;
}

/*! \copybrief XsDevice::updateRateForDataIdentifier
*/
int MtDevice::updateRateForDataIdentifier(XsDataIdentifier dataType) const
{
	return XsDevice::updateRateForDataIdentifier(dataType);
}

/*! \copybrief XsDevice::stringOutputType
*/
uint16_t MtDevice::stringOutputType() const
{
	XsMessage snd(XMID_ReqStringOutputType), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::stringSamplePeriod
*/
uint16_t MtDevice::stringSamplePeriod() const
{
	XsMessage snd(XMID_ReqPeriod), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::stringSkipFactor
*/
uint16_t MtDevice::stringSkipFactor() const
{
	XsMessage snd(XMID_ReqOutputSkipFactor), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::deviceOptionFlags
*/
XsDeviceOptionFlag MtDevice::deviceOptionFlags() const
{
	XsMessage snd(XMID_ReqOptionFlags), rcv;
	if (doTransaction(snd, rcv))
		return (XsDeviceOptionFlag)rcv.getDataLong();
	return XDOF_None;
}
/*! \copybrief XsDevice::gnssPlatform
*/
XsGnssPlatform MtDevice::gnssPlatform() const
{
	XsMessage snd(XMID_ReqGnssPlatform), rcv;
	if (doTransaction(snd, rcv))
		return (XsGnssPlatform)rcv.getDataShort();
	return XGP_Portable;
}

/*! \copybrief XsDevice::outputConfiguration
*/
XsOutputConfigurationArray MtDevice::outputConfiguration() const
{
	return XsOutputConfigurationArray();
}

/*! \brief Checks if this device can do orientation reset in firmware
	\param method The reset method
	\returns True if successful
*/
bool MtDevice::canDoOrientationResetInFirmware(XsResetMethod method)
{
	switch (method)
	{
	case XRM_DefaultAlignment:
	case XRM_DefaultHeading:
	case XRM_DefaultInclination:
		return true;

	case XRM_None:
		return false;

	default:
		break;
	}

	return updateRateForDataIdentifier(XDI_OrientationGroup) > 0;
}

/*! \copybrief XsDevice::scheduleOrientationReset */
bool MtDevice::scheduleOrientationReset(XsResetMethod method)
{
	switch (deviceState()) {
	case XDS_Measurement:
	case XDS_Recording:
		if (method == XRM_StoreAlignmentMatrix)
			return false;

		if (canDoOrientationResetInFirmware(method))
			if (!XsDevice::scheduleOrientationReset(method))
				return false;

		break;

	case XDS_Config:
		if (method != XRM_StoreAlignmentMatrix)
			return false;

		if (canDoOrientationResetInFirmware(method))
		{
			if (!storeAlignmentMatrix())
				return false;
			// read stored value from emts by reinitializing
			return reinitialize();
		}
		return true;

	default:
		return false;
	}
	return true;
}

/*! \brief Store the current alignment matrix in the device.
	\details The alignment matrix is computed when doing an orientation reset and needs to be stored
	explicitly in the device or it will be forgotten when the device restarts. This function will tell
	the device to store its alignment matrix or to write the locally computed alignment matrix to the
	device when filtering is done on the PC side.
	\returns true if the alignment matrix was successfully written to the non-volatile memory of the
	device
*/
bool MtDevice::storeAlignmentMatrix()
{
	if (!XsDevice::scheduleOrientationReset(XRM_StoreAlignmentMatrix))
		return false;

	return true;
}

/*! \brief The heading offset set for this device
*/
double MtDevice::headingOffset() const
{
	XsMessage snd(XMID_ReqHeading), rcv;
	snd.setBusId(busId());

	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataFloat();
}

/*! \copybrief XsDevice::setLocationId
*/
bool MtDevice::setLocationId(int id)
{
	XsMessage snd(XMID_SetLocationId, XS_LEN_LOCATIONID);
	snd.setBusId(busId());
	snd.setDataShort((uint16_t)id);

	return doTransaction(snd);
}

/*! \copybrief XsDevice::locationId
*/
int MtDevice::locationId() const
{
	XsMessage snd(XMID_ReqLocationId), rcv;
	snd.setBusId(busId());

	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::dataLength
*/
int MtDevice::dataLength() const
{
	return deviceConfiguration().deviceInfo(deviceId()).m_dataLength;
}

/*! \copybrief XsDevice::serialBaudRate
*/
XsBaudRate MtDevice::serialBaudRate() const
{
	XsMessage snd(XMID_ReqBaudrate), rcv;
	snd.setBusId(busId());

	if (!doTransaction(snd, rcv))
		return XBR_Invalid;

	return (XsBaudRate)rcv.getDataByte();
}

/*! \copybrief XsDevice::hardwareVersion
*/
XsVersion MtDevice::hardwareVersion() const
{
	XsMessage snd(XMID_ReqHardwareVersion), rcv;
	if (!doTransaction(snd, rcv))
		return XsVersion();
	uint16_t hwv = rcv.getDataShort();
	return XsVersion(hwv >> 8, hwv & 0xff);
}

/*! \copybrief XsDevice::availableOnboardFilterProfiles
*/
std::vector<XsFilterProfile> MtDevice::availableOnboardFilterProfiles() const
{
	return m_hardwareFilterProfiles;
}

/*!	\brief Request the filter profiles headers from the hardware device and returns a vector with the found profiles.
	the order in the output vector is the same as the order in the hardware device.
*/
std::vector<XsFilterProfile> MtDevice::readFilterProfilesFromDevice() const
{
	std::vector<XsFilterProfile> result;

	XsMessage snd(XMID_ReqAvailableFilterProfiles);
	snd.setBusId(busId());

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return result;

	uint8_t filterType = deviceConfiguration().deviceInfo(deviceId()).m_filterType;

	XsSize nofScenarios = rcv.getDataSize() / (1 + 1 + XS_LEN_FILTERPROFILELABEL);

	result.resize(nofScenarios);
	for (XsSize i = 0; i < nofScenarios; ++i)
	{
		result[i].setType(rcv.getDataByte(0 + i*(1+1+XS_LEN_FILTERPROFILELABEL)));
		result[i].setVersion(rcv.getDataByte(1 + i*(1+1+XS_LEN_FILTERPROFILELABEL)));
		result[i].setLabel((const char*) rcv.getDataBuffer(2 + i*(1+1+XS_LEN_FILTERPROFILELABEL)));
		result[i].setFilterType(filterType);
	}
	return result;
}

/*! \brief Fetches available hardware scenarios
*/
void MtDevice::fetchAvailableHardwareScenarios()
{
	m_hardwareFilterProfiles.clear();
	m_hardwareFilterProfiles = readFilterProfilesFromDevice();
	std::sort(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(), CompareXsFilterProfile());
}

/*! \copybrief XsDevice::productCode
*/
XsString MtDevice::productCode() const
{
	XsMessage snd(XMID_ReqProductCode), rcv;
	if (!doTransaction(snd, rcv))
		return XsString();

	const char* pc = (const char*) rcv.getDataBuffer();
	std::string result(pc?pc:"", 20);
	std::string::size_type thingy = result.find(" ");
	if (thingy < 20)
		result.erase(result.begin() + thingy, result.end());
	return XsString(result);
}

/*! \copybrief XsDevice::reinitialize
*/
bool MtDevice::reinitialize()
{
	if (!readDeviceConfiguration())
		return false;

	clearDataCache();
	fetchAvailableHardwareScenarios();
	return true;
}

/*! \brief Restore to factory default settings
*/
bool MtDevice::restoreFactoryDefaults()
{
	if (!XsDevice::restoreFactoryDefaults())
		return false;

	return reinitialize();
}

/*! \copybrief XsDevice::onboardFilterProfile
*/
XsFilterProfile MtDevice::onboardFilterProfile() const
{
	return m_hardwareFilterProfile;
}

/*! \copybrief XsDevice::setOnboardFilterProfile
*/
bool MtDevice::setOnboardFilterProfile(int profileType)
{
	return setOnboardFilterProfile(XsFilterProfile((uint8_t) profileType));
}

/*! \copybrief XsDevice::setOnboardFilterProfile
*/
bool MtDevice::setOnboardFilterProfile(const XsFilterProfile &scenario)
{
	if (deviceState() != XDS_Config)
		return false;

	std::vector<XsFilterProfile>::iterator item = std::find_if(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(), ScenarioMatchPred(scenario));
	if (item == m_hardwareFilterProfiles.end())
		return false;

	XsMessage snd(XMID_SetFilterProfile, XS_LEN_SETFILTERPROFILE);
	snd.setBusId(busId());
	snd.setDataShort(scenario.type());

	if (!doTransaction(snd))
		return false;

	m_hardwareFilterProfile = *item;
	return true;
}

/*! \returns The accelerometer range for this device
	\details The range is an absolute maximum number. This means that if 100 is returned, the sensor range is (100, -100)
*/
double MtDevice::accelerometerRange() const
{
	return ::accelerometerRange(productCode(), hardwareVersion().major());
}

/*! \returns The accelerometer range for this device
	\details The range is an absolute maximum number. This means that if 300 is returned, the sensor range is (300, -300)
*/
double MtDevice::gyroscopeRange() const
{
	return ::gyroscopeRange(productCode());
}

/*! \brief Write the emts of the device to the open logfile
	\note The default implementation of MtDevice does not include any children
*/
void MtDevice::writeDeviceSettingsToFile()
{
	writeMessageToLogFile(m_emtsBlob);
}

/*! \copybrief XsDevice::setNoRotation
*/
bool MtDevice::setNoRotation(uint16_t duration)
{
	XsMessage snd(XMID_SetNoRotation, 2);
	snd.setBusId(busId());
	snd.setDataShort(duration);

	return doTransaction(snd);
}

/*!	\brief Set the current sensor position.
	\details Use this function to set the current position in latitude, longitude, altitude.
	\param lla The LLA vector
	\returns True if successful
	\sa initialPositionLLA
*/
bool MtDevice::setInitialPositionLLA(const XsVector& lla)
{
	uint8_t bid = busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST || lla.size() != 3)
		return false;

	XsMessage snd(XMID_SetLatLonAlt, XS_LEN_LATLONALT);
	snd.setDataFloat((float) lla[0], 0);
	snd.setDataFloat((float) lla[1], 4);
	snd.setDataFloat((float) lla[2], 8);
	snd.setBusId(bid);

	return doTransaction(snd);
}

/*! \returns the current sensor position
	\sa setInitialPositionLLA
*/
XsVector MtDevice::initialPositionLLA() const
{
	XsMessage snd(XMID_ReqLatLonAlt), rcv;
	if (doTransaction(snd, rcv))
	{
		XsVector3 vec;
		for (int i = 0; i < 3; i++)
			vec[i] = rcv.getDataDouble(i * 8);
		return vec;
	}
	return XsVector();
}

/*! \brief Convert mt sync ticks to microseconds
*/
uint32_t MtDevice::syncTicksToUs(uint32_t ticks) const
{
	return ((uint32_t) (((double) ticks) * XS_SYNC_CLOCK_TICKS_TO_US + 0.5));
}

/*! \brief Convert microseconds to mt sync ticks
*/
uint32_t MtDevice::usToSyncTicks(uint32_t us) const
{
	return ((uint32_t) (((double) us) * XS_SYNC_CLOCK_US_TO_TICKS + 0.5));
}

/*! \returns the error mode of the device.
	\see setErrorMode
*/
XsErrorMode MtDevice::errorMode() const
{
	XsMessage snd(XMID_ReqErrorMode), rcv;
	if (!doTransaction(snd, rcv))
		return XEM_Ignore;
	return static_cast<XsErrorMode>(rcv.getDataShort());
}

/*! \brief Set the error mode of the device
	\details The error mode determines how the device handles errors. See
	the low-level communication documentation for more details.
	\param em The error mode
	\returns True if successful
	\see errorMode
*/
bool MtDevice::setErrorMode(XsErrorMode em)
{
	XsMessage snd(XMID_SetErrorMode, 2);
	snd.setBusId(busId());
	snd.setDataShort(em);
	return doTransaction(snd);
}

/*! \brief Return the RS485 acknowledge transmission delay of the device
	\details The RS485 acknowledge transmission delay determines the minimal
	delay to response on request messages. See the low-level communication
	documentation for more details.
	\returns The delay value
	\see setRs485TransmissionDelay()
*/
uint16_t MtDevice::rs485TransmissionDelay() const
{
	XsMessage snd(XMID_ReqTransmitDelay), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \brief Set the RS485 acknowledge transmission delay of the device
	\param delay The delay value
	\returns True if successful
	\see rs485TransmissionDelay()
*/
bool MtDevice::setRs485TransmissionDelay(uint16_t delay)
{
	XsMessage snd(XMID_SetTransmitDelay, 2);
	snd.setBusId(busId());
	snd.setDataShort(delay);

	return doTransaction(snd);
}

/*! \brief Request data from the motion tracker
	\details The reply is handled by the live data path.
	This functionality is only available if outputSkipFactor() is set to 0xffff.
	\returns True if successful
*/
bool MtDevice::requestData()
{
	XsMessage snd(XMID_ReqData);
	snd.setBusId(busId());

	return sendRawMessage(snd);
}

/*! \brief Run a self test
	\returns The self test result
*/
XsSelfTestResult MtDevice::runSelfTest()
{
	XsMessage snd(XMID_RunSelfTest, 0);
	snd.setBusId(busId());
	XsMessage rcv;
	if (!doTransaction(snd, rcv, 3000))
		return XsSelfTestResult();

	return XsSelfTestResult::create(rcv.getDataShort());
}

/*! \copybrief XsDevice::storeFilterState
*/
bool MtDevice::storeFilterState()
{
	if (deviceState() == XDS_Config)
	{
		XsMessage snd(XMID_StoreFilterState);
		snd.setBusId(busId());

		if (doTransaction(snd))
			return true;
	}
//	m_lastResult = XRV_INVALIDOPERATION;
	return false;
}

/*! \brief Calculates the frequency
	\param baseFrequency The base frequency to calculate with
	\param skipFactor The skip factor to calculate with
	\returns The calculated frequency
*/
int MtDevice::calcFrequency(int baseFrequency, uint16_t skipFactor)
{
	int result = baseFrequency / (skipFactor + 1);
	return result;
}

///@{ \name Log files
/*!
	\brief Set the read position of the open log file to the start of the file.

	If software filtering is enabled, the appropriate filters will be restarted
	as if the file was just opened. This function sets the lastResult value
	to indicate success (XRV_OK) or failure.

	\return True if successful
	\sa lastResult()
	\note This is a low-level file operation.
	\internal
*/
bool MtDevice::resetLogFileReadPosition()
{
	JLDEBUGG("");

	if (!XsDevice::resetLogFileReadPosition())
		return false;

	return true;
}

///@} end Log files
