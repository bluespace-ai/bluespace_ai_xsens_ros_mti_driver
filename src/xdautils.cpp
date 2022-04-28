#include "xdautils.hpp"

#include <map>
#include <iostream>
#include <sstream>

std::string get_xs_data_identifier_name(XsDataIdentifier identifier)
{
  switch (identifier) {
    case XDI_TemperatureGroup: return "XDI_TemperatureGroup";
    case XDI_Temperature: return "XDI_Temperature";

    case XDI_TimestampGroup: return "XDI_TimestampGroup";
    case XDI_UtcTime: return "XDI_UtcTime";
    case XDI_PacketCounter: return "XDI_PacketCounter";
    case XDI_Itow: return "XDI_Itow";
    case XDI_GnssAge: return "XDI_GnssAge";
    case XDI_PressureAge: return "XDI_PressureAge";
    case XDI_SampleTimeFine: return "XDI_SampleTimeFine";
    case XDI_SampleTimeCoarse: return "XDI_SampleTimeCoarse";
    case XDI_FrameRange: return "XDI_FrameRange";
    case XDI_PacketCounter8: return "XDI_PacketCounter8";
    case XDI_SampleTime64: return "XDI_SampleTime64";

    case XDI_OrientationGroup: return "XDI_OrientationGroup";
    case XDI_Quaternion: return "XDI_Quaternion";
    case XDI_RotationMatrix: return "XDI_RotationMatrix";
    case XDI_EulerAngles: return "XDI_EulerAngles";

    case XDI_PressureGroup: return "XDI_PressureGroup";
    case XDI_BaroPressure: return "XDI_BaroPressure";

    case XDI_AccelerationGroup: return "XDI_AccelerationGroup";
    case XDI_DeltaV: return "XDI_DeltaV";
    case XDI_Acceleration: return "XDI_Acceleration";
    case XDI_FreeAcceleration: return "XDI_FreeAcceleration";
    case XDI_AccelerationHR: return "XDI_AccelerationHR";

    case XDI_IndicationGroup: return "XDI_IndicationGroup";
    case XDI_TriggerIn1: return "XDI_TriggerIn1";
    case XDI_TriggerIn2: return "XDI_TriggerIn2";
    case XDI_TriggerIn3: return "XDI_TriggerIn3";

    case XDI_PositionGroup: return "XDI_PositionGroup";
    case XDI_AltitudeMsl: return "XDI_AltitudeMsl";
    case XDI_AltitudeEllipsoid: return "XDI_AltitudeEllipsoid";
    case XDI_PositionEcef: return "XDI_PositionEcef";
    case XDI_LatLon: return "XDI_LatLon";

    case XDI_GnssGroup: return "XDI_GnssGroup";
    case XDI_GnssPvtData: return "XDI_GnssPvtData";
    case XDI_GnssSatInfo: return "XDI_GnssSatInfo";
    case XDI_GnssPvtPulse: return "XDI_GnssPvtPulse";

    case XDI_AngularVelocityGroup: return "XDI_AngularVelocityGroup";
    case XDI_RateOfTurn: return "XDI_RateOfTurn";
    case XDI_DeltaQ: return "XDI_DeltaQ";
    case XDI_RateOfTurnHR: return "XDI_RateOfTurnHR";

    case XDI_RawSensorGroup: return "XDI_RawSensorGroup";
    case XDI_RawAccGyrMagTemp: return "XDI_RawAccGyrMagTemp";
    case XDI_RawGyroTemp: return "XDI_RawGyroTemp";
    case XDI_RawAcc: return "XDI_RawAcc";
    case XDI_RawGyr: return "XDI_RawGyr";
    case XDI_RawMag: return "XDI_RawMag";
    case XDI_RawDeltaQ: return "XDI_RawDeltaQ";
    case XDI_RawDeltaV: return "XDI_RawDeltaV";
    case XDI_RawBlob: return "XDI_RawBlob";

    case XDI_AnalogInGroup: return "XDI_AnalogInGroup";
    case XDI_AnalogIn1: return "XDI_AnalogIn1";
    case XDI_AnalogIn2: return "XDI_AnalogIn2";

    case XDI_MagneticGroup: return "XDI_MagneticGroup";
    case XDI_MagneticField: return "XDI_MagneticField";
    case XDI_MagneticFieldCorrected: return "XDI_MagneticFieldCorrected";

    case XDI_SnapshotGroup: return "XDI_SnapshotGroup";
    case XDI_AwindaSnapshot: return "XDI_AwindaSnapshot";
    case XDI_FullSnapshot: return "XDI_FullSnapshot";
    case XDI_GloveSnapshotLeft: return "XDI_GloveSnapshotLeft";
    case XDI_GloveSnapshotRight: return "XDI_GloveSnapshotRight";

    case XDI_GloveDataGroup: return "XDI_GloveDataGroup";
    case XDI_GloveDataLeft: return "XDI_GloveDataLeft";
    case XDI_GloveDataRight: return "XDI_GloveDataRight";

    case XDI_VelocityGroup: return "XDI_VelocityGroup";
    case XDI_VelocityXYZ: return "XDI_VelocityXYZ";

    case XDI_StatusGroup: return "XDI_StatusGroup";
    case XDI_StatusByte: return "XDI_StatusByte";
    case XDI_StatusWord: return "XDI_StatusWord";
    case XDI_Rssi: return "XDI_Rssi";
    case XDI_DeviceId: return "XDI_DeviceId";
    case XDI_LocationId: return "XDI_LocationId";
    default:    return "???";
  }
}

bool get_xs_data_identifier_by_name(const std::string & name, XsDataIdentifier & identifier)
{
  std::map<std::string, XsDataIdentifier> name_mapping;

  name_mapping["XDI_TemperatureGroup"] = XDI_TemperatureGroup;
  name_mapping["XDI_Temperature"] = XDI_Temperature;

  name_mapping["XDI_TimestampGroup"] = XDI_TimestampGroup;
  name_mapping["XDI_UtcTime"] = XDI_UtcTime;
  name_mapping["XDI_PacketCounter"] = XDI_PacketCounter;
  name_mapping["XDI_Itow"] = XDI_Itow;
  name_mapping["XDI_GnssAge"] = XDI_GnssAge;
  name_mapping["XDI_PressureAge"] = XDI_PressureAge;
  name_mapping["XDI_SampleTimeFine"] = XDI_SampleTimeFine;
  name_mapping["XDI_SampleTimeCoarse"] = XDI_SampleTimeCoarse;
  name_mapping["XDI_FrameRange"] = XDI_FrameRange;
  name_mapping["XDI_PacketCounter8"] = XDI_PacketCounter8;
  name_mapping["XDI_SampleTime64"] = XDI_SampleTime64;

  name_mapping["XDI_OrientationGroup"] = XDI_OrientationGroup;
  name_mapping["XDI_Quaternion"] = XDI_Quaternion;
  name_mapping["XDI_RotationMatrix"] = XDI_RotationMatrix;
  name_mapping["XDI_EulerAngles"] = XDI_EulerAngles;

  name_mapping["XDI_PressureGroup"] = XDI_PressureGroup;
  name_mapping["XDI_BaroPressure"] = XDI_BaroPressure;

  name_mapping["XDI_AccelerationGroup"] = XDI_AccelerationGroup;
  name_mapping["XDI_DeltaV"] = XDI_DeltaV;
  name_mapping["XDI_Acceleration"] = XDI_Acceleration;
  name_mapping["XDI_FreeAcceleration"] = XDI_FreeAcceleration;
  name_mapping["XDI_AccelerationHR"] = XDI_AccelerationHR;

  name_mapping["XDI_IndicationGroup"] = XDI_IndicationGroup;
  name_mapping["XDI_TriggerIn1"] = XDI_TriggerIn1;
  name_mapping["XDI_TriggerIn2"] = XDI_TriggerIn2;
  name_mapping["XDI_TriggerIn3"] = XDI_TriggerIn3;

  name_mapping["XDI_PositionGroup"] = XDI_PositionGroup;
  name_mapping["XDI_AltitudeMsl"] = XDI_AltitudeMsl;
  name_mapping["XDI_AltitudeEllipsoid"] = XDI_AltitudeEllipsoid;
  name_mapping["XDI_PositionEcef"] = XDI_PositionEcef;
  name_mapping["XDI_LatLon"] = XDI_LatLon;

  name_mapping["XDI_GnssGroup"] = XDI_GnssGroup;
  name_mapping["XDI_GnssPvtData"] = XDI_GnssPvtData;
  name_mapping["XDI_GnssSatInfo"] = XDI_GnssSatInfo;
  name_mapping["XDI_GnssPvtPulse"] = XDI_GnssPvtPulse;

  name_mapping["XDI_AngularVelocityGroup"] = XDI_AngularVelocityGroup;
  name_mapping["XDI_RateOfTurn"] = XDI_RateOfTurn;
  name_mapping["XDI_DeltaQ"] = XDI_DeltaQ;
  name_mapping["XDI_RateOfTurnHR"] = XDI_RateOfTurnHR;

  name_mapping["XDI_RawSensorGroup"] = XDI_RawSensorGroup;
  name_mapping["XDI_RawAccGyrMagTemp"] = XDI_RawAccGyrMagTemp;
  name_mapping["XDI_RawGyroTemp"] = XDI_RawGyroTemp;
  name_mapping["XDI_RawAcc"] = XDI_RawAcc;
  name_mapping["XDI_RawGyr"] = XDI_RawGyr;
  name_mapping["XDI_RawMag"] = XDI_RawMag;
  name_mapping["XDI_RawDeltaQ"] = XDI_RawDeltaQ;
  name_mapping["XDI_RawDeltaV"] = XDI_RawDeltaV;
  name_mapping["XDI_RawBlob"] = XDI_RawBlob;

  name_mapping["XDI_AnalogInGroup"] = XDI_AnalogInGroup;
  name_mapping["XDI_AnalogIn1"] = XDI_AnalogIn1;
  name_mapping["XDI_AnalogIn2"] = XDI_AnalogIn2;

  name_mapping["XDI_MagneticGroup"] = XDI_MagneticGroup;
  name_mapping["XDI_MagneticField"] = XDI_MagneticField;
  name_mapping["XDI_MagneticFieldCorrected"] = XDI_MagneticFieldCorrected;

  name_mapping["XDI_SnapshotGroup"] = XDI_SnapshotGroup;
  name_mapping["XDI_AwindaSnapshot"] = XDI_AwindaSnapshot;
  name_mapping["XDI_FullSnapshot"] = XDI_FullSnapshot;
  name_mapping["XDI_GloveSnapshotLeft"] = XDI_GloveSnapshotLeft;
  name_mapping["XDI_GloveSnapshotRight"] = XDI_GloveSnapshotRight;

  name_mapping["XDI_GloveDataGroup"] = XDI_GloveDataGroup;
  name_mapping["XDI_GloveDataLeft"] = XDI_GloveDataLeft;
  name_mapping["XDI_GloveDataRight"] = XDI_GloveDataRight;

  name_mapping["XDI_VelocityGroup"] = XDI_VelocityGroup;
  name_mapping["XDI_VelocityXYZ"] = XDI_VelocityXYZ;

  name_mapping["XDI_StatusGroup"] = XDI_StatusGroup;
  name_mapping["XDI_StatusByte"] = XDI_StatusByte;
  name_mapping["XDI_StatusWord"] = XDI_StatusWord;
  name_mapping["XDI_Rssi"] = XDI_Rssi;
  name_mapping["XDI_DeviceId"] = XDI_DeviceId;
  name_mapping["XDI_LocationId"] = XDI_LocationId;

  auto value = name_mapping.find(name);
  if (value != name_mapping.end()) {
    identifier = value->second;
    return true;
  } else {
    return false;
  }
}

bool parseConfigLine(const std::string & line, std::string & name, int & value)
{
  std::istringstream stream;
  stream.str(line);
  int index = 0;
  for (std::string part; std::getline(stream, part, '='); index++) {
    switch (index) {
      case 0:
        name = part;
        break;
      case 1:
        value = std::stoi(part);
        break;
      default:
        return false;
    }
  }

  return index == 2;
}
