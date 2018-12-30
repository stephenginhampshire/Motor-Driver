/* 
 * File:   ASCOM_Fields.h
 * Author: Stephen
 *
 * Created on 01 December 2017, 17:03
 */

#ifndef ASCOM_FIELDS_H
#define	ASCOM_FIELDS_H

//------------------------------------------------------------------------------
//double Altitude;                 // heart beat
//double Azimuth;                  // heart beat
//union sg_long ASCOM_QED_Counter;
//------------------------------------------------------------------------------
//bool Slewing = false;                           // status A0
//bool Tracking = false;                          // ststus A1
//bool Homeing = false;                           // status A2
//bool AtPark = false;                            // status A6
//bool AtHome = false;                            // status A7

//------------------------------------------------------------------------------
union sg_double_union Declination;
union sg_double_union RightAscension;
union sg_double_union DeclinationRate;
union sg_double_union FocalLength;
union sg_double_union TrackingRate;
union sg_double_union TargetRightAscension;
union sg_double_union TargetDeclination;
union sg_double_union SlewSettleTime;
union sg_double_union SiteLongitude;
union sg_double_union SiteLatitude;
//double SiderealTime = 0;
union sg_double_union RightAscensionRate;

union sg_double_union GuideRateRightAscension;
union sg_double_union GuideRateDeclination;
union sg_double_union SiteElevation;
union sg_double_union ApertureDiameter;
union sg_double_union ApertureArea;
union sg_double_union TrackingRates;
//------------------------------------------------------------------------------
bool CanUnpark = false;
bool CanSyncAltAz = false;
bool CanSync = false;
bool CanSlewAsync = false;
bool CanSlewAltAz = false;
bool CanSlew = false;
bool CanSetTracking = false;
bool CanSetRightAscensionRate = false;
bool CanSetPierSide = false;
bool CanSetPark = false;
bool CanSetGuideRates = false;
bool CanSetDeclinationRate = false;
bool CanPulseGuide = false;
bool DoesRefraction = false;
char EquatorialSystem = 0;
bool CanPark = 0;
bool IsPulseGuiding = false;
bool CanFindHome = false;
// DateTime UTCDate = 0;

char AlignmentMode = 0;
//char SupportedActions[];
char InterfaceVersion = 0x01;
char DriverVersion = 0x01;
char DriverInfo = 0x01;
#ifdef ALTITUDE_MOTOR
char Description[] = {"AltitudeMotorDriver"};
#else
char Description[] = ("AzimuthMotorDriver");
#endif
bool Connected = false;
bool CanMoveAxis = false;
//------------------------------------------------------------------------------
void AbortSlew(void);
// string Action(string ActionName, string ActionParameters);
// IAxisRates AxisRates(TelescopeAxes Axis);

// void CommandBlind(string Command, bool Raw = false);
// bool CommandBool(string Command, bool Raw = false);
// string CommandString(string Command, bool Raw = false);
// PierSide DestinationSideOfPier(double RightAscension, double Declination);
// void FindHome(void);
// void MoveAxis(int Axis, double Rate);
// void Park(void);
// void PulseGuide(int Direction, int Duration);
// void SetPark(void);
//void SetupDialog();
void SlewToAltAz(double Altitude, double Azimuth);
// void SlewToAltAzAsync(double Altitude, double Azimuth);
// void SlewToCoordinates(double RightAscension, double Declination);
// void SlewToTarget();
// void SlewToTargetAsync();
// void SyncToAltAz(double Azimuth, double Altitude);
// void SyncToCoordinates(double RightAscension, double Declination);
// void SyncToTarget();
// void Unpark(void);
#endif	/* ASCOM_FIELDS_H */

