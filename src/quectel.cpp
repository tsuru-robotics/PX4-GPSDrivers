/****************************************************************************
 *
 *   Copyright (c) 2020, 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file quectel.cpp
 *
 * Quectel protocol implementation.
 *
 * @author Vladimir Savelyev <vms@flyfire.io>
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>

#include "quectel.h"
#include "rtcm.h"

#ifndef M_PI_F
# define M_PI_F 3.14159265358979323846f
#endif

#define MIN(X,Y)              ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y)    ((X) > (Y) ? (X) : (Y))
#define QL_UNUSED(x) (void)x;

/**** Warning macros, disable to save memory */
#define QL_WARN(...)         {GPS_WARN(__VA_ARGS__);}
#define QL_DEBUG(...)        {/*GPS_INFO(__VA_ARGS__);*/}

// PQTM messages
static constexpr char QL_PQTM_MSG_NAME_GEOFENCESTATUS[] {"PQTMGEOFENCESTATUS"};
static constexpr char QL_PQTM_MSG_NAME_EPE[] {"PQTMEPE"};
static constexpr char QL_PQTM_MSG_NAME_PVT[] {"PQTMPVT"};
static constexpr char QL_PQTM_MSG_NAME_PL[]  {"PQTMPL"};
static constexpr char QL_PQTM_MSG_NAME_DOP[] {"PQTMDOP"};
static constexpr char QL_PQTM_MSG_NAME_VEL[] {"PQTMVEL"};
static constexpr char QL_PQTM_MSG_NAME_ODO[] {"PQTMODO"};

GPSDriverQL::GPSDriverQL(GPSCallbackPtr callback, void *callback_user,
			     sensor_gps_s *gps_position,
			     satellite_info_s *satellite_info):
	GPSHelper(callback, callback_user),
	_gps_position(gps_position),
	_satellite_info(satellite_info)
{
	decodeInit();
}

GPSDriverQL::~GPSDriverQL()
{
	delete _rtcm_parsing;
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

int GPSDriverQL::handleMessage(int len)
{
	char *endp;

	if (len < 7) {
		return 0;
	}

	int uiCalcComma = 0;

	for (int i = 0 ; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}

	char *bufptr = (char *)(_rx_buffer + 6);
	int ret = 0;

	if ((memcmp(_rx_buffer + 3, "GGA,", 4) == 0) && (uiCalcComma >= 14)) {
		/*
		  Time, position, and fix related data
		  An example of the GBS message string is:
		  $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
		  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
		  $GNGGA,092721.00,2926.688113,N,11127.771644,E,2,08,1.11,106.3,M,-20,M,1.0,3721*53

		  Note - The data string exceeds the nmea standard length.
		  GGA message fields
		  Field   Meaning
		  0   Message ID $GPGGA
		  1   UTC of position fix
		  2   Latitude
		  3   Direction of latitude:
		  N: North
		  S: South
		  4   Longitude
		  5   Direction of longitude:
		  E: East
		  W: West
		  6   GPS Quality indicator:
		  0: Fix not valid
		  1: GPS fix
		  2: Differential GPS fix, OmniSTAR VBS
		  4: Real-Time Kinematic, fixed integers
		  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
		  7   Number of SVs in use, range from 00 through to 24+
		  8   HDOP
		  9   Orthometric height (MSL reference)
		  10  M: unit of measure for orthometric height is meters
		  11  Geoid separation
		  12  M: geoid separation measured in meters
		  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
		  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
		  15
		  The checksum data, always begins with *
		*/
		double utc_time = 0.0, lat = 0.0, lon = 0.0;
		float alt = 0.f, geoid_h = 0.f;
		float hdop = 99.9f, dgps_age = NAN;
		int  num_of_sv = 0, fix_quality = 0;
		char ns = '?', ew = '?';

		QL_UNUSED(utc_time);
		QL_UNUSED(alt);
		QL_UNUSED(lat);
		QL_UNUSED(lon);
		QL_UNUSED(geoid_h);
		QL_UNUSED(hdop);
		QL_UNUSED(num_of_sv);

		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt = strtof(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip M

		if (bufptr && *(++bufptr) != ',') { geoid_h = strtof(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip M

		if (bufptr && *(++bufptr) != ',') { dgps_age = strtof(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		_gps_position->gga_dgps_age = dgps_age;

		if (fix_quality <= 0) {
			_gps_position->fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (fix_quality == 5) { fix_quality = 3; }

			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position->fix_type = 3 + fix_quality - 1;
		}

		QL_DEBUG("--handled GGA");

	} else if ((memcmp(_rx_buffer + 3, "GSV,", 4) == 0)) {
		/*
		The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

		$GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

		GSV message fields
		Field   Meaning
		0   Message ID $GPGSV
		1   Total number of messages of this type in this cycle
		2   Message number
		3   Total number of SVs visible
		4   SV PRN number
		5   Elevation, in degrees, 90 maximum
		6   Azimuth, degrees from True North, 000 through 359
		7   SNR, 00 through 99 dB (null when not tracking)
		8-11    Information about second SV, same format as fields 4 through 7
		12-15   Information about third SV, same format as fields 4 through 7
		16-19   Information about fourth SV, same format as fields 4 through 7
		20  The checksum data, always begins with *
		*/

		int all_page_num = 0, this_page_num = 0, tot_sv_visible = 0;
		struct gsv_sat {
			int svid;
			int elevation;
			int azimuth;
			int snr;
		} sat[4] {};

		if (bufptr && *(++bufptr) != ',') { all_page_num = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { this_page_num = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

		if ((this_page_num < 1) || (this_page_num > all_page_num)) {
			QL_WARN("GSV parse error. this_page_num not valid");
			return 0;
		}

		if (this_page_num == 0 && _satellite_info) {
			// initialize sat info with zeros
			memset(_satellite_info, 0, sizeof(*_satellite_info));
		}

		int end = 4;

		if (this_page_num == all_page_num) {
			end =  tot_sv_visible - (this_page_num - 1) * 4;

			_SVINFO_received = true;

			if (_satellite_info) {
				_satellite_info->count = MIN(tot_sv_visible, satellite_info_s::SAT_INFO_MAX_SATELLITES);
				_gps_position->timestamp = gps_absolute_time();
				_last_timestamp_time = _gps_position->timestamp;
			}
		}

		if (_satellite_info) {
			if ((end < 0) || (end > 4)) {
				QL_WARN("GSV parse error. amount of satellites not valid");
				return 0;
			}
			QL_DEBUG("GSV: parsing page %d/%d containig %d satellites info", this_page_num, all_page_num, end);
			for (int y = 0 ; y < end ; y++) {

				int sat_index = y + (this_page_num - 1) * 4;

				if ((sat_index < 0) || (sat_index > satellite_info_s::SAT_INFO_MAX_SATELLITES)) {
					QL_WARN("GSV parse error. sat_index %d not valid", sat_index);
					return 0;
				}

				if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

				_satellite_info->svid[sat_index]      = sat[y].svid;
				_satellite_info->used[sat_index]      = (sat[y].snr > 0);
				_satellite_info->snr[sat_index]       = sat[y].snr;
				_satellite_info->elevation[sat_index] = sat[y].elevation;
				_satellite_info->azimuth[sat_index]   = sat[y].azimuth;

				QL_DEBUG("GSV: added satellite id %d to satellite_info[%d]", sat[y].svid, sat_index);
			}
		}

		QL_DEBUG("--handled GSV");

	} else if ((memcmp(_rx_buffer, "$PQTMPVT,", 9) == 0) && (uiCalcComma >= 19)) {
		/*
		  $PQTMPVT,MsgVer,TOW,Date,Time,Res,FixMode,NumSatUsed,LeapS,Lat,Lon,Alt,Sep,
		  VelN,VelE,VelD,Spd,Heading,HDOP,PDOP*Checksum<CR><LF>
		*/
		double utc_time = 0.0, lat = 0.0, lon = 0.0;
		float alt = 0.f, sep = 0.f;
		float vel_n = 0.f, vel_e = 0.f, vel_d = 0.f, spd = 0.f;
		int  num_of_satellites = 0;
		int nmea_date = 0;


		/* Set buffer pointer to data (size of "$PQTMPVT," == 9)*/
		bufptr = (char *)(_rx_buffer + 9);

		while (*(++bufptr) != ',') {} //skip MsgVer
		while (*(++bufptr) != ',') {} //skip TOW

		/* Extract <Date> */
		if (bufptr && *(++bufptr) != ',') { nmea_date = static_cast<int>(strtol(bufptr, &endp, 10)); bufptr = endp; }

		/* Extract <Time> */
		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip Res
		while (*(++bufptr) != ',') {} //skip FixMode

		/* Extract <NumSatUsed> */
		if (bufptr && *(++bufptr) != ',') { num_of_satellites = strtol(bufptr, &endp, 10); bufptr = endp; }

		/* skip LeapS */
		while (*(++bufptr) != ',') {}

		/* Extract <Lat> */
		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		/* Extract <Lon> */
		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		/* Extract <Alt> */
		if (bufptr && *(++bufptr) != ',') { alt = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <Sep> */
		if (bufptr && *(++bufptr) != ',') { sep = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <VelN> */
		if (bufptr && *(++bufptr) != ',') { vel_n = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <VelE> */
		if (bufptr && *(++bufptr) != ',') { vel_e = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <VelD> */
		if (bufptr && *(++bufptr) != ',') { vel_d = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <Spd> */
		if (bufptr && *(++bufptr) != ',') { spd = strtof(bufptr, &endp); bufptr = endp; }

		/* Skip <Heading> */
		while (*(++bufptr) != ',') {}

		/* Skip <HDOP> */
		while (*(++bufptr) != ',') {}

		// Position
		_gps_position->lon = static_cast<int>((int(lon * 10000000)));
		_gps_position->lat = static_cast<int>((int(lat * 10000000)));
		_POS_received = true;

		// Altitude
		_gps_position->alt = static_cast<int>(alt * 1000);
		_gps_position->alt_ellipsoid = static_cast<int>((alt + sep) * 1000);

		// Velocity
		_gps_position->vel_m_s = spd;
		_gps_position->vel_n_m_s = vel_n;
		_gps_position->vel_e_m_s = vel_e;
		_gps_position->vel_d_m_s = vel_d;
		/**< Flag to indicate if NED speed is valid */
		_gps_position->vel_ned_valid = true;
		_VEL_received = true;

		// DOP - do not use DOP from this message since VDOP not present
		// _gps_position->hdop = hdop;

		// Course over ground
		// DO NOT SET since no variance available in message
		/*
		float track_rad = heading * M_PI_F / 180.0f; // rad in range [0, 2pi]
		if (track_rad > M_PI_F)
		{
			track_rad -= 2.f * M_PI_F; // rad in range [-pi, pi]
		}
		_gps_position->cog_rad = track_rad;
		_gps_position->c_variance_rad = 0.1f;
		*/

		// Satellites used
		_gps_position->satellites_used = num_of_satellites;

		// Timestamp
		int utc_hour = static_cast<int>(utc_time / 10000);
		int utc_minute = static_cast<int>((utc_time - utc_hour * 10000) / 100);
		double utc_sec = static_cast<double>(utc_time - utc_hour * 10000 - utc_minute * 100);
		int nmea_year = static_cast<int>(nmea_date / 10000);
		int nmea_mth = static_cast<int>((nmea_date - nmea_year * 10000) / 100);
		int nmea_day= static_cast<int>(nmea_date - nmea_year * 10000 - nmea_mth * 100);

		/*
		 * convert to unix timestamp
		 */
		struct tm timeinfo = {};
		timeinfo.tm_year = nmea_year - 1900;
		timeinfo.tm_mon = nmea_mth - 1;
		timeinfo.tm_mday = nmea_day;
		timeinfo.tm_hour = utc_hour;
		timeinfo.tm_min = utc_minute;
		timeinfo.tm_sec = int(utc_sec);
		timeinfo.tm_isdst = 0;

#ifndef NO_MKTIME
		time_t epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((utc_sec - static_cast<uint64_t>(utc_sec)) * 1000000);

			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.
			timespec ts{};
			ts.tv_sec = epoch;
			ts.tv_nsec = usecs * 1000;
			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += usecs;

		} else {
			_gps_position->time_utc_usec = 0;
		}

#else
		_gps_position->time_utc_usec = 0;
#endif
		_gps_position->timestamp = gps_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;

		QL_DEBUG("--handled PQTMPVT");

	} else if ((memcmp(_rx_buffer, "$PQTMVEL,", 9) == 0) && (uiCalcComma >= 11)) {
		/*
		  $PQTMVEL,1,<Time>,<VelN>,<VelE>,<VelD>,<GrdSpd>,<Spd>,
		  <Heading>,<GrdSpdAcc>,<SpdAcc>,<HeadingAcc>*<Checksum><CR><LF>
		*/
		float vel_n = 0.f, vel_e = 0.f, vel_d = 0.f, grd_spd = 0.f;
		float heading = 0.f;
		float heading_acc = 99.9f;
		float spd_acc = 99.9f;


		/* Set buffer pointer to data (size of "$PQTMVEL," == 9)*/
		bufptr = (char *)(_rx_buffer + 9);

		while (*(++bufptr) != ',') {} //skip 1 (MsgVer?)
		while (*(++bufptr) != ',') {} //skip Time

		/* Extract <VelN> */
		if (bufptr && *(++bufptr) != ',') { vel_n = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <VelE> */
		if (bufptr && *(++bufptr) != ',') { vel_e = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <VelD> */
		if (bufptr && *(++bufptr) != ',') { vel_d = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <GrdSpd> */
		if (bufptr && *(++bufptr) != ',') { grd_spd = strtof(bufptr, &endp); bufptr = endp; }

		/* Skip <Spd> */
		while (*(++bufptr) != ',') {}

		/* Extract <Heading> // Course over ground */
		if (bufptr && *(++bufptr) != ',') { heading = strtof(bufptr, &endp); bufptr = endp; }

		/* Skip <GrdSpdAcc> */
		while (*(++bufptr) != ',') {}

		/* Extract <SpdAcc> */
		if (bufptr && *(++bufptr) != ',') { spd_acc = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <HeadingAcc> // Course over ground accuracy */
		if (bufptr && *(++bufptr) != ',') { heading_acc = strtof(bufptr, &endp); bufptr = endp; }

		// Course over ground
		float track_rad = heading * M_PI_F / 180.0f; // rad in range [0, 2pi]
		float track_rad_accuracy = heading_acc * M_PI_F / 180.0f;

		if (track_rad > M_PI_F)
		{
			track_rad -= 2.f * M_PI_F; // rad in range [-pi, pi]
		}
		_gps_position->cog_rad = track_rad;
		_gps_position->c_variance_rad = track_rad_accuracy;

		// Velocity
		_gps_position->vel_m_s = grd_spd;
		_gps_position->vel_n_m_s = vel_n;
		_gps_position->vel_e_m_s = vel_e;
		_gps_position->vel_d_m_s = vel_d;
		_gps_position->s_variance_m_s = spd_acc;
		/**< Flag to indicate if NED speed is valid */
		_gps_position->vel_ned_valid = true;

		_VEL_received = true;

		// Should also fill in timestamp for vel according to SensorGps.msg
		_gps_position->timestamp = gps_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;

		QL_DEBUG("--handled PQTMVEL");

	} else if ((memcmp(_rx_buffer, "$PQTMEPE,", 9) == 0) && (uiCalcComma >= 6)) {
		/*
		  $PQTMEPE,2,<EPE_North>,<EPE_East>,<EPE_Down>,
		  <EPE_2D>,<EPE_3D>*<Checksum><CR><LF>
		*/
		float epe_2d = 99.9f;
		float epe_down = 99.9f;

		/* Set buffer pointer to data (size of "$PQTMVEL," == 9)*/
		bufptr = (char *)(_rx_buffer + 9);

		while (*(++bufptr) != ',') {} //skip 2 (MsgVer?)
		while (*(++bufptr) != ',') {} //skip EPE_North
		while (*(++bufptr) != ',') {} //skip EPE_East

		/* Extract <EPE_Down> */
		if (bufptr && *(++bufptr) != ',') { epe_down = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <EPE_2D> */
		if (bufptr && *(++bufptr) != ',') { epe_2d = strtof(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip EPE_3D

		// EPH and EPV
		_gps_position->eph = epe_2d;
		_gps_position->epv = epe_down;

		QL_DEBUG("--handled PQTMEPE");

	} else if ((memcmp(_rx_buffer, "$PQTMDOP,", 9) == 0) && (uiCalcComma >= 9)) {
		/*
		  $PQTMDOP,<MsgVer>,<TOW>,<GDOP>,<PDOP>,<TDOP>,
		  <VDOP>,<HDOP>,<NDOP>,<EDOP>*<Checksum><CR><LF>
		*/
		float hdop = 99.9f;
		float vdop = 99.9f;

		/* Set buffer pointer to data (size of "$PQTMVEL," == 9)*/
		bufptr = (char *)(_rx_buffer + 9);

		while (*(++bufptr) != ',') {} //skip MsgVer
		while (*(++bufptr) != ',') {} //skip TOW
		while (*(++bufptr) != ',') {} //skip GDOP
		while (*(++bufptr) != ',') {} //skip PDOP
		while (*(++bufptr) != ',') {} //skip TDOP

		/* Extract <VDOP> */
		if (bufptr && *(++bufptr) != ',') { vdop = strtof(bufptr, &endp); bufptr = endp; }

		/* Extract <HDOP> */
		if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip NDOP
		while (*(++bufptr) != ',') {} //skip EDOP

		// DOP
		_gps_position->hdop = hdop;
		_gps_position->vdop = vdop;

		QL_DEBUG("--handled PQTMDOP");

	} else if ((memcmp(_rx_buffer, "$PAIRSPF5,", 10) == 0) && (uiCalcComma == 1)) {
		/*
		$PAIRSPF5,0*66

		Field	Meaning
		0	Message ID $PAIRSPF
		1   	Jamming status.
		*/

		uint8_t status = 0;

		/* Set buffer pointer to data (size of "$PAIRSPF5," == 10)*/
		bufptr = (char *)(_rx_buffer + 10);

		if (bufptr && *(++bufptr) != ',') { status = strtol(bufptr, &endp, 10); bufptr = endp; }

		_gps_position->jamming_l5_state = status;

		QL_DEBUG("--handled PAIRSPF5");

	}  else if ((memcmp(_rx_buffer, "$PAIRSPF,", 9) == 0) && (uiCalcComma == 1)) {
		/*
		$PAIRSPF,0*53

		Field	Meaning
		0	Message ID $PAIRSPF
		1  	Jamming status.
		*/

		uint8_t status = 0;

		/* Set buffer pointer to data (size of "$PAIRSPF," == 9)*/
		bufptr = (char *)(_rx_buffer + 9);

		if (bufptr && *(++bufptr) != ',') { status = strtol(bufptr, &endp, 10); bufptr = endp; }

		_gps_position->jamming_l1_state = status;

		QL_DEBUG("--handled PAIRSPF");

	} else if ((memcmp(_rx_buffer, "$PAIR001,", 9) == 0) && (uiCalcComma == 2)) {
		/*
		PAIR_ACK
		$PAIR001,<CommandID>,<Result>*<Checksum><CR><LF>
		*/

		/* Set buffer pointer to data (size of "$PAIR001," == 9)*/
		bufptr = (char *)(_rx_buffer + 9);

		if (bufptr && *(++bufptr) != ',') { _ack_nmea_command_id     = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { _ack_nmea_command_result = strtol(bufptr, &endp, 10); bufptr = endp; }

		_ACK_received = true;

		QL_DEBUG("--handled PAIR001");

	}else if ((memcmp(_rx_buffer, "$PQTMCFGMSGRATE,OK", 18) == 0) && (uiCalcComma == 1)) {
		/*
		Response to Set command:
		$PQTMCFGMSGRATE,OK*<Checksum><CR><LF>
		*/

		_ack_pqtm_command = true;
		_ack_pqtm_command_error_code = 0;
		_ACK_received = true;

		QL_DEBUG("--handled PQTMCFGMSGRATE,OK");

	} else if ((memcmp(_rx_buffer, "$PQTMCFGMSGRATE,ERROR,", 22) == 0) && (uiCalcComma == 2)) {
		/*
		Response to Set command:
		$PQTMCFGMSGRATE,ERROR,<ErrCode>*<Checksum><CR><LF>
		*/

		_ack_pqtm_command = false;

		/* Set buffer pointer to ErrCode*/
		bufptr = (char *)(_rx_buffer + 22);

		if (bufptr && *(++bufptr) != ',') { _ack_pqtm_command_error_code = strtol(bufptr, &endp, 10); bufptr = endp; }
		_ACK_received = true;

		QL_DEBUG("--handled PQTMCFGMSGRATE,ERROR");

	} else if ((memcmp(_rx_buffer, "$PQTMDEBUGOFF,OK", 16) == 0) && (uiCalcComma == 1)) {
		/*
		Response to Set command:
		$PQTMCFGMSGRATE,OK*<Checksum><CR><LF>
		*/

		_ack_pqtm_command = true;
		_ack_pqtm_command_error_code = 0;
		_ACK_received = true;

		QL_DEBUG("--handled PQTMDEBUGOFF,OK");

	} else if ((memcmp(_rx_buffer, "$PQTMDEBUGOFF,ERROR,", 20) == 0) && (uiCalcComma == 2)) {
		/*
		Response to Set command:
		$PQTMDEBUGOFF,ERROR,<ErrCode>*<Checksum><CR><LF>
		*/

		_ack_pqtm_command = false;

		/* Set buffer pointer to ErrCode*/
		bufptr = (char *)(_rx_buffer + 20);

		if (bufptr && *(++bufptr) != ',') { _ack_pqtm_command_error_code = strtol(bufptr, &endp, 10); bufptr = endp; }
		_ACK_received = true;

		QL_DEBUG("--handled PQTMDEBUGOFF,ERROR");
	}

	if (_waiting_for_ACK) {
		if (_ACK_received) {
			ret = 1;
			_ACK_received = false;
		}

	} else {

		if (_POS_received && _VEL_received) {
			ret = 1;
		}

		if (_POS_received) {
			_POS_received = false;
			_rate_count_lat_lon++;
		}

		if (_VEL_received) {
			_VEL_received  = false;
			_rate_count_vel++;
		}

		if (_SVINFO_received) {
			ret = 2;
			_SVINFO_received = false;
		}
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverQL::receive(unsigned timeout)
{
	QL_DEBUG("Start receive with timeout %u ms, waiting for ack = %d", timeout, _waiting_for_ACK);
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {
		int ret = read(buf, sizeof(buf), timeout);
		QL_DEBUG("--read %d bytes", ret);

		if (ret < 0) {
			/* something went wrong when polling or reading */
			QL_WARN("Receive poll or read err");
			return -1;

		} else if (ret != 0) {

			/* pass received bytes to the packet decoder */
			for (int i = 0; i < ret; i++) {
				int parsed_msg_length = parseChar(buf[i]);

				if (parsed_msg_length > 0) {
					handled |= handleMessage(parsed_msg_length);
				}
			}

			if (handled > 0) {
				QL_DEBUG("Receive finished successfully.");
				return handled;
			}else {
				QL_DEBUG("--handle res=%d ", handled);
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			QL_WARN("Timeout receive");
			return -1;
		}
	}
}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int GPSDriverQL::parseChar(uint8_t b)
{
	int parsed_msg_length = 0;

	switch (_decode_state) {
	/* First, look for sync1 */
	case NMEADecodeState::uninit:
		if (b == '$') {
			_decode_state = NMEADecodeState::got_sync1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;

		}  else if (b == RTCM3_PREAMBLE && _rtcm_parsing) {
			_decode_state = NMEADecodeState::decode_rtcm3;
			_rtcm_parsing->addByte(b);

		}

		break;

	case NMEADecodeState::got_sync1:
		if (b == '$') {
			_decode_state = NMEADecodeState::got_sync1;
			_rx_buffer_bytes = 0;

		} else if (b == '*') {
			_decode_state = NMEADecodeState::got_asteriks;
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
			_decode_state = NMEADecodeState::uninit;
			_rx_buffer_bytes = 0;

		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NMEADecodeState::got_asteriks:
		_rx_buffer[_rx_buffer_bytes++] = b;
		_decode_state = NMEADecodeState::got_first_cs_byte;
		break;

	case NMEADecodeState::got_first_cs_byte: {
			_rx_buffer[_rx_buffer_bytes++] = b;
			uint8_t checksum = 0;
			uint8_t *buffer = _rx_buffer + 1;
			uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

			for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

			if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
			    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
				parsed_msg_length = _rx_buffer_bytes;
			} else {
				QL_WARN("CRC mismatch");
			}

			QL_DEBUG("--parsed: %.*s", _rx_buffer_bytes, _rx_buffer);

			decodeInit();
		}
		break;

	case NMEADecodeState::decode_rtcm3:
		if (_rtcm_parsing->addByte(b)) {
			QL_DEBUG("got RTCM message with length %i", (int)_rtcm_parsing->messageLength());
			gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
			decodeInit();
		}

		break;
	}

	return parsed_msg_length;
}

void GPSDriverQL::decodeInit()
{
	_rx_buffer_bytes = 0;
	_decode_state = NMEADecodeState::uninit;

	if (_output_mode == OutputMode::GPSAndRTCM || _output_mode == OutputMode::RTCM) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		if (_rtcm_parsing) {
			_rtcm_parsing->reset();
		}
	}
}

int GPSDriverQL::configure(unsigned &baudrate, const GPSConfig &config)
{
	QL_DEBUG("Configuring GPS ...");

	_output_mode = config.output_mode;

	if (_output_mode != OutputMode::GPS) {
		QL_WARN("RTCM output has to be configured manually");
	}

	if (baudrate > 0) {

		QL_DEBUG("--baudrate set to %i", baudrate);
		setBaudrate(baudrate);

		/* flush input and wait for at least 20 ms silence */
		decodeInit();
		receive(20);
		decodeInit();

		// configure messages
		bool msg_rates_configured = configMessages(config.quectel_msg_config);

		// receive valid messages with position and velocity
		int ret = receive(500);

		if (msg_rates_configured && (ret > 0)) {
			QL_DEBUG("GPS configured, baudrate=%d", baudrate);
			return 0;
		} else {
			QL_WARN("Failed configuring GPS");
			return -1;
		}
	} else {
		QL_WARN("Failed configuring GPS, baudrate is set to 0");
		return -1;
	}
}

int
GPSDriverQL::reset(GPSRestartType restart_type)
{
	switch (restart_type) {
	case GPSRestartType::Hot:
		if (reset_hot()) {
			return 0;
		}
		break;

	case GPSRestartType::Warm:
		if (reset_warm()) {
			return 0;
		}
		break;

	case GPSRestartType::Cold:
		if (reset_cold()) {
			return 0;
		}
		break;

	default:
		break;
	}

	return -2;
}

bool
GPSDriverQL::configMessages(const QlMsgConfig &config)
{
	// Set GGA rate (Output once every N position fix(es))
	if (!setNmeaMsgOutputRate(QlNmeaMsgId::GGA, config.RateGGA)) {
		QL_WARN("Failed configuring GGA rate %d", config.RateGGA);
		return false;
	}
	// Set GSV rate (optional), output once every position fix)
	unsigned gsv_rate = (_satellite_info) ? 1 : 0;
	if (!setNmeaMsgOutputRate(QlNmeaMsgId::GSV, gsv_rate)) {
		QL_WARN("Failed configuring GSV rate %d", gsv_rate);
		return false;
	}
	// Disable GLL
	if (!setNmeaMsgOutputRate(QlNmeaMsgId::GLL, 0)) {
		QL_WARN("Failed disabling GLL");
		return false;
	}
	// Disable GSA
	if (!setNmeaMsgOutputRate(QlNmeaMsgId::GSA, 0)) {
		QL_WARN("Failed disabling GSA");
		return false;
	}
	// Disable RMC
	if (!setNmeaMsgOutputRate(QlNmeaMsgId::RMC, 0)) {
		QL_WARN("Failed disabling RMC");
		return false;
	}
	// Disable VTG
	if (!setNmeaMsgOutputRate(QlNmeaMsgId::VTG, 0)) {
		QL_WARN("Failed disabling VTG");
		return false;
	}
	// Set PQTMPVT rate (Output once every position fix)
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_PVT, 1, QlPqtmMsgVer::VER1)) {
		QL_WARN("Failed enabling PQTMPVT");
		return false;
	}
	// Set PQTMEPE rate (Output once every N position fix(es))
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_EPE, config.RatePQTMEPE, QlPqtmMsgVer::VER2)) {
		QL_WARN("Failed configuring PQTMEPE rate %d", config.RatePQTMEPE);
		return false;
	}
	// Set PQTMDOP rate (Output once every N position fix(es))
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_DOP, config.RatePQTMDOP, QlPqtmMsgVer::VER1)) {
		QL_WARN("Failed configuring PQTMDOP rate %d", config.RatePQTMDOP);
		return false;
	}
	// Set PQTMVEL rate (Output once every N position fix(es))
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_VEL, config.RatePQTMVEL, QlPqtmMsgVer::VER1)) {
		QL_WARN("Failed configuring PQTMVEL rate %d", config.RatePQTMVEL);
		return false;
	}
	// Disable PQTMPL
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_PL, 0, QlPqtmMsgVer::VER1)) {
		QL_WARN("Failed disabling PQTMPL");
		return false;
	}
	// Disable PQTMGEOFENCESTATUS
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_GEOFENCESTATUS, 0, QlPqtmMsgVer::VER1)) {
		QL_WARN("Failed disabling PQTMGEOFENCESTATUS");
		return false;
	}
	// Disable PQTMODO
	if (!setPqtmMsgOutputRate(QL_PQTM_MSG_NAME_ODO, 0, QlPqtmMsgVer::VER1)) {
		QL_WARN("Failed disabling PQTMODO");
		return false;
	}
	// Set PQTM debug logging mode
	if (!setPqtmDebugMode(config.PQTMDebugMode)) {
		QL_WARN("Failed configuring PQTM log mode %d", config.PQTMDebugMode);
		return false;
	}
	// Set NMEA debug logging mode
	if (!setNmeaDebugMode(config.NmeaDebugMode)) {
		QL_WARN("Failed configuring NMEA log mode %d", config.NmeaDebugMode);
		return false;
	}

	return true;
}

bool
GPSDriverQL::setNmeaMsgOutputRate(QlNmeaMsgId nmea_msg_type, unsigned msg_rate)
{
	char msg[QL_OUT_MSG_MAX_SIZE] = "";
	snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PAIR062,%d,%d*", static_cast<int>(nmea_msg_type), msg_rate);

	if (!writeMessage(msg)) {
		return false;
	}

	int res = waitForNmeaAck(62, QL_CONFIG_TIMEOUT);
	// Expected result is "Command has been successfully sent."
	if (res != 0) {
		QL_WARN("Failed waiting ACK for PAIR062 with error=%d", res);
		return false;
	}

	return true;
}

bool
GPSDriverQL::setNmeaDebugMode(unsigned mode)
{
	char msg[QL_OUT_MSG_MAX_SIZE] = "";

	snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PAIR086,%d*", mode);

	if (!writeMessage(msg)) {
		return false;
	}

	int res = waitForNmeaAck(86, QL_CONFIG_TIMEOUT);
	// Expected result is "Command has been successfully sent."
	if (res != 0) {
		QL_WARN("Failed waiting ACK for PAIR086 with error=%d", res);
		return false;
	}

	return true;
}

bool
GPSDriverQL::reset_hot()
{
	char msg[QL_OUT_MSG_MAX_SIZE] = "";
	snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PAIR004*");

	if (!writeMessage(msg)) {
		return false;
	}

	int res = waitForNmeaAck(4, QL_RESET_ACK_TIMEOUT);
	// Expected "Command has been successfully sent" or "Command is being processed"
	if ((res == 0) || (res == 1)){
		return true;
	} else {
		QL_WARN("Failed waiting ACK for PAIR004 with error=%d", res);
		return false;
	}
}

bool
GPSDriverQL::reset_warm()
{
	char msg[QL_OUT_MSG_MAX_SIZE] = "";
	snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PAIR005*");

	if (!writeMessage(msg)) {
		return false;
	}

	int res = waitForNmeaAck(5, QL_RESET_ACK_TIMEOUT);
	// Expected "Command has been successfully sent" or "Command is being processed"
	if ((res == 0) || (res == 1)){
		return true;
	} else {
		QL_WARN("Failed waiting ACK for PAIR005 with error=%d", res);
		return false;
	}
}

bool
GPSDriverQL::reset_cold()
{
	char msg[QL_OUT_MSG_MAX_SIZE] = "";
	snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PAIR006*");

	if (!writeMessage(msg)) {
		return false;
	}

	int res = waitForNmeaAck(6, QL_RESET_ACK_TIMEOUT);
	// Expected "Command has been successfully sent" or "Command is being processed"
	if ((res == 0) || (res == 1)){
		return true;
	} else {
		QL_WARN("Failed waiting ACK for PAIR006 with error=%d", res);
		return false;
	}
}

int
GPSDriverQL::waitForNmeaAck(uint8_t command_id, unsigned timeout)
{
	_waiting_for_ACK = true;
	receive(timeout);
	_waiting_for_ACK = false;

	int res = -1; // timeout

	if (_ack_nmea_command_id == command_id) {
		res = _ack_nmea_command_result;
		_ack_nmea_command_id = 0;
		_ack_nmea_command_result = 0;
	}

	return res;
}

bool
GPSDriverQL::setPqtmMsgOutputRate(const char pqtm_msg_name[], unsigned msg_rate, QlPqtmMsgVer msg_ver)
{
	char msg[QL_OUT_MSG_MAX_SIZE] = "";
	if (msg_ver > QlPqtmMsgVer::NONE) {
		snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PQTMCFGMSGRATE,W,%s,%d,%d*", pqtm_msg_name, msg_rate, static_cast<int>(msg_ver));
	} else {
		snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PQTMCFGMSGRATE,W,%s,%d*", pqtm_msg_name, msg_rate);
	}

	if (!writeMessage(msg)) {
		return false;
	}

	return waitForPqtmAck(msg, QL_CONFIG_TIMEOUT);
}

bool
GPSDriverQL::setPqtmDebugMode(unsigned mode)
{
	if (mode == 0) {
		char msg[QL_OUT_MSG_MAX_SIZE] = "";

		snprintf(msg, QL_OUT_MSG_MAX_SIZE, "$PQTMDEBUGOFF*");

		if (!writeMessage(msg)) {
			return false;
		}

		return waitForPqtmAck(msg, QL_CONFIG_TIMEOUT);
	} else {
		return true;
	}
}

bool
GPSDriverQL::waitForPqtmAck(char msg[QL_OUT_MSG_MAX_SIZE], unsigned timeout)
{
	_waiting_for_ACK = true;
	receive(timeout);
	_waiting_for_ACK = false;

	bool res = false;
	if (!_ack_pqtm_command) {
		QL_WARN("Failed waiting ACK for %s with error=%d", msg, _ack_pqtm_command_error_code);
	} else {
		res = true;
	}

	_ack_pqtm_command = false;
	_ack_pqtm_command_error_code = 0;

	return res;
}

int GPSDriverQL::calcChecksum(const char *msg, size_t msg_length, char* checksum)
{
	unsigned char sum = 0;

	if((nullptr == msg) || (msg_length < 1)) {
		return -1;
	}

	for(size_t i = 1; (i < msg_length) && (msg[i] != '\0') && (msg[i] != '*'); i++) { // Skip the '$'
		sum ^= msg[i];
	}

	snprintf(checksum, 3, "%02X", sum);

	return 0;
}

bool
GPSDriverQL::writeMessage(char msg[QL_OUT_MSG_MAX_SIZE])
{
	char checksum[3] = "";
	if (calcChecksum(msg, strlen(msg), checksum) < 0) {
		QL_WARN("Checksum calculation failed for msg %s", msg);
		return false;
	}
	QL_DEBUG("Add to msg %s checksum %s", msg, checksum);
	strncat(msg, checksum, QL_OUT_MSG_MAX_SIZE - strlen(msg) - 1);
	QL_DEBUG("Add to msg %s '\r\n'", msg);
	strncat(msg, "\r\n",    QL_OUT_MSG_MAX_SIZE - strlen(msg) - 1);

	size_t msg_size = strlen(msg);
	if (write((void *)msg, msg_size) != (int)msg_size) {
		QL_WARN("Error writing %s of size %d", msg, (unsigned)msg_size);
		return false;
	} else {
		QL_DEBUG("Written %s of size %d", msg, (unsigned)msg_size);
	}

	return true;
}
