/**
 * @file quectel.cpp
 *
 * @author quectel
 */

#include "quectel.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <math.h>

#include "rtcm.h"

#define MAX(X,Y)    ((X) > (Y) ? (X) : (Y))
#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

#define GNSS_SYSTEM_ID_GPS        1
#define	GNSS_SYSTEM_ID_GLONASS    2
#define	GNSS_SYSTEM_ID_GALILEO    3
#define	GNSS_SYSTEM_ID_BDS        4
#define	GNSS_SYSTEM_ID_QZSS       5


GPSDriverQL::GPSDriverQL(GPSCallbackPtr callback, void* callback_user,
                        sensor_gps_s* gps_position,
                        satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _gps_position(gps_position),
    _satellite_info(satellite_info)
{
    decodeInit();
}

int
GPSDriverQL::configure(unsigned& baudrate, const GPSConfig& config)
{

    return 0;
}

int
GPSDriverQL::receive(unsigned timeout)
{
    uint8_t buf[QL_RX_BUFF_LENGTH]{ 0 };
    /* timeout additional to poll */
    gps_abstime time_started = gps_absolute_time();

    int handled = 0;

    while (true) {

        int ret = read(buf, sizeof(buf), timeout);

        if (ret > 0)
        {
            for (int i = 0; i < ret; i++) {

                int len = parseChar(buf[i]);
                QL_DEBUG("Parsed %d chars from %d bytes", len, ret);
                if (len > 0) {
                    handled |= handleMessage(len);
                }
            }

            if (handled > 0) {
                return handled;
            }
        }

        if (ret < 0)
        {
            QL_ERR("Read error %d", ret);
            return -1;
        }

        /* in case we keep trying but only get crap from GPS */
        if (time_started + timeout * 1000 < gps_absolute_time()) {
            QL_ERR("in case we keep trying but only get crap from GPS");
            return -1;
        }
    }
}

void
GPSDriverQL::decodeInit()
{
    _rx_ck_a = 0;
    _rx_ck_b = 0;
    _rx_count = 0;
    _decode_state = QL_DECODE_UNINIT;
}
void
GPSDriverQL::msgFlagInit()
{
    _sat_num_gga = 0;
    _sat_num_gns = 0;
    _sat_num_gsv = 0;
    _sat_num_gpgsv = 0;
    _sat_num_glgsv = 0;
    _sat_num_gagsv = 0;
    _sat_num_gbgsv = 0;
    _sat_num_gqgsv = 0;

    _TIME_received = false;
    _POS_received = false;
    _ALT_received = false;
    _SVNUM_received = false;
    _SVINFO_received = false;
    _FIX_received = false;
    _DOP_received = false;
    _VEL_received = false;
    _EPH_received = false;
    _HEAD_received = false;

    _is_frame_end = false;

    memset(_gps_used_svid, 0, sizeof(_gps_used_svid));
    memset(_gln_used_svid, 0, sizeof(_gln_used_svid));
    memset(_bds_used_svid, 0, sizeof(_bds_used_svid));
    memset(_gal_used_svid, 0, sizeof(_gal_used_svid));
    memset(_qzss_used_svid, 0, sizeof(_qzss_used_svid));
}

int
GPSDriverQL::parseChar(uint8_t b)
{
    int iRet = 0;

    switch (_decode_state) {
        /* First, look for sync */
        case QL_DECODE_UNINIT:
            if (b == '$') {
                _decode_state = QL_DECODE_GOT_SYNC;
                _rx_count = 0;
                _rx_buffer[_rx_count++] = b;

            }
            //else if (b == RTCM3_PREAMBLE && _rtcm_parsing) {
            //	_decode_state = QL_DECODE_RTCM3;
            //	_rtcm_parsing->addByte(b);

            //}

            break;

        case QL_DECODE_GOT_SYNC:
            if (b == '$') {
                _decode_state = QL_DECODE_GOT_SYNC;
                _rx_count = 0;

            }
            else if (b == '*') {
                _decode_state = QL_DECODE_GOT_ASTERIK;
            }

            if (_rx_count >= (sizeof(_rx_buffer) - 5)) {
                _decode_state = QL_DECODE_UNINIT;
                _rx_count = 0;

            }
            else {
                _rx_buffer[_rx_count++] = b;
            }

            break;

        case QL_DECODE_GOT_ASTERIK:
            _rx_buffer[_rx_count++] = b;
            _decode_state = QL_DECODE_GOT_FIRST_CS_BYTE;
            break;

        case QL_DECODE_GOT_FIRST_CS_BYTE: {
            _rx_buffer[_rx_count++] = b;
            uint8_t checksum = 0;
            uint8_t* buffer = _rx_buffer + 1;
            uint8_t* bufend = _rx_buffer + _rx_count - 3;

            for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

            if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_count - 2)) &&
                (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_count - 1))) {
                iRet = _rx_count;
            }

            decodeInit();
        }
            break;

        case QL_DECODE_RTCM3:
        	// if (_rtcm_parsing->addByte(b)) {
        	// 	snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "got RTCM message with length %i", (int)_rtcm_parsing->messageLength());
        	// 	gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
        	// 	decodeInit();
        	// }
        	break;
    }

    return iRet;
}

bool
GPSDriverQL::is_same_nmea_msg_id(int offset, const char* msg_id)
{
    int id_length = 0;

    char* commaPos = std::strchr((char*)_rx_buffer, ',');
    if (commaPos == NULL) {
        return false;
    }

    /* $PAIR319,$PQTM,$XXGGA */
    id_length = commaPos - (char*)_rx_buffer;

    if (memcmp(_rx_buffer + offset, msg_id, id_length - offset) == 0){
        return true;
    }

    return false;
}
int
GPSDriverQL::handleMessage(int packet_len)
{
    _rx_buffer[packet_len] = 0;
    // snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "Need decode packet: %s\r\n", _rx_buffer);

    if (strstr((char*)_rx_buffer, "PQTM") != nullptr) {
        QL_DEBUG("PQTM");

        char* bufptr = std::strchr((char*)_rx_buffer, ',');

        if (is_same_nmea_msg_id(1, "PQTMVEL"))
        {
            decode_msg_pqtmvel(bufptr);
        }
        else if (is_same_nmea_msg_id(1, "PQTMEPE"))
        {
            decode_msg_pqtmepe(bufptr);
        }
        else
        {
            char* commaPos = std::strchr((char*)_rx_buffer, ',');
            if (commaPos != NULL)
            {
                *commaPos = '\0';
            }
            QL_WARN("Not found packet: %s \r\n", _rx_buffer);
        }
    }
    else if (strstr((char*)_rx_buffer, "PAIR") != nullptr) {
        QL_DEBUG("PAIR");

        char* bufptr = std::strchr((char*)_rx_buffer, ',');

        if (is_same_nmea_msg_id(1, "PAIRSPF"))
        {
            decode_msg_pairspf(bufptr);
        }
        else if (is_same_nmea_msg_id(1, "PAIRSPF5"))
        {
            decode_msg_pairspf5(bufptr);
            _is_frame_end = true;
        }
        else
        {
            char* commaPos = std::strchr((char*)_rx_buffer, ',');
            if (commaPos != NULL)
            {
                *commaPos = '\0';
            }
            QL_WARN("Not found packet: %s \r\n", _rx_buffer);
        }
    }
    else {

        QL_DEBUG("ELSE");

        char* bufptr = (char*)(_rx_buffer + 6);

        if (is_same_nmea_msg_id(3, "GGA")) {
            decode_msg_gga(bufptr);
        }
        else if (is_same_nmea_msg_id(3, "RMC")) {
            decode_msg_rmc(bufptr);
        }
        else if (is_same_nmea_msg_id(3, "GSA")) {
            decode_msg_gsa(bufptr);
        }
        else if (is_same_nmea_msg_id(3, "GSV")) {
            decode_msg_gsv(bufptr);
        }
        else if (is_same_nmea_msg_id(3, "VTG")) {
            decode_msg_vtg(bufptr);
        }
        else {
            char* commaPos = std::strchr((char*)_rx_buffer, ',');
            if (commaPos != NULL)
            {
                *commaPos = '\0';
            }
            QL_WARN("Not found packet: %s \r\n", _rx_buffer);
        }
    }

    if (_is_frame_end)
    {
        msgFlagInit();
        return 1;
    }

    return 0;
}

bool
GPSDriverQL::is_used_svid(uint8_t svid, uint8_t* used_svid)
{
    uint8_t* ptr = used_svid;

    if (ptr == nullptr) {
        return false;
    }

    while (*ptr != 0) {
        if (*ptr == svid) {
            return true;
        }
        ptr++;
    }

    return false;
}

int
GPSDriverQL::decode_msg_gga(char* bufptr)
{
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
    char* endp;
    double utc_time = 0.0, lat = 0.0, lon = 0.0;
    float alt = 0.f, geoid_h = 0.f;
    float hdop = 99.9f;//, dgps_age = NAN;
    int  num_of_sv = 0, fix_quality = 0;
    char ns = '?', ew = '?';

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

    if (bufptr && *(++bufptr) != ',') { /*dgps_age = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (ns == 'S') {
        lat = -lat;
    }

    if (ew == 'W') {
        lon = -lon;
    }

    /* convert from degrees, minutes and seconds to degrees */
    _gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
    _gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
    _gps_position->hdop = hdop;
    _gps_position->alt = static_cast<int>(alt * 1000);
    _gps_position->alt_ellipsoid = _gps_position->alt + static_cast<int>(geoid_h * 1000);
    _sat_num_gga = static_cast<int>(num_of_sv);

    if (fix_quality <= 0) {
        _gps_position->fix_type = 0;
    }
    else {
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

    if (!_POS_received && (_last_POS_timeUTC < utc_time)) {
        _last_POS_timeUTC = utc_time;
        _POS_received = true;
    }

    _ALT_received = true;
    _SVNUM_received = true;
    _FIX_received = true;

    _gps_position->c_variance_rad = 0.1f;
    _gps_position->timestamp = gps_absolute_time();

    return 1;
}

int
GPSDriverQL::decode_msg_rmc(char* bufptr)
{
    /*
    Position, velocity, and time
    The RMC string is:

    $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
    The Talker ID ($--) will vary depending on the satellite system used for the position solution:
    $GNRMC,092721.00,A,2926.688113,N,11127.771644,E,0.780,,200520,,,D,V*1D

    GPRMC message fields
    Field	Meaning
    0	Message ID $GPRMC
    1	UTC of position fix
    2	Status A=active or V=void
    3	Latitude
    4	Longitude
    5	Speed over the ground in knots
    6	Track angle in degrees (True)
    7	Date
    8	Magnetic variation in degrees
    9	The checksum data, always begins with *
    */
    char* endp;
    double utc_time = 0.0;
    char Status = 'V';
    double lat = 0.0, lon = 0.0;
    float ground_speed_K = 0.f;
    float track_true = 0.f;
    int nmea_date = 0;
    //float Mag_var = 0.f;
    char ns = '?', ew = '?';

    if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { Status = *(bufptr++); }

    if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

    if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

    if (bufptr && *(++bufptr) != ',') { ground_speed_K = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { track_true = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { nmea_date = static_cast<int>(strtol(bufptr, &endp, 10)); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { /*Mag_var = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (ns == 'S') {
        lat = -lat;
    }

    if (ew == 'W') {
        lon = -lon;
    }

    if (Status == 'V') {
        _gps_position->fix_type = 0;
    }

    float track_rad = track_true * M_PI_F / 180.0f; // rad in range [0, 2pi]

    if (track_rad > M_PI_F) {
        track_rad -= 2.f * M_PI_F; // rad in range [-pi, pi]
    }

    float velocity_ms = ground_speed_K / 1.9438445f;
    float velocity_north = velocity_ms * cosf(track_rad);
    float velocity_east = velocity_ms * sinf(track_rad);

    /* convert from degrees, minutes and seconds to degrees */
    _gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
    _gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);

    _gps_position->vel_m_s = velocity_ms;
    _gps_position->vel_n_m_s = velocity_north;
    _gps_position->vel_e_m_s = velocity_east;
    _gps_position->cog_rad = track_rad;
    _gps_position->vel_ned_valid = true; /**< Flag to indicate if NED speed is valid */
    _gps_position->c_variance_rad = 0.1f;
    _gps_position->s_variance_m_s = 0;
    _gps_position->timestamp = gps_absolute_time();
    _last_timestamp_time = gps_absolute_time();

#ifndef NO_MKTIME
    int utc_hour = static_cast<int>(utc_time / 10000);
    int utc_minute = static_cast<int>((utc_time - utc_hour * 10000) / 100);
    double utc_sec = static_cast<double>(utc_time - utc_hour * 10000 - utc_minute * 100);
    int nmea_day = static_cast<int>(nmea_date / 10000);
    int nmea_mth = static_cast<int>((nmea_date - nmea_day * 10000) / 100);
    int nmea_year = static_cast<int>(nmea_date - nmea_day * 10000 - nmea_mth * 100);
    /*
     * convert to unix timestamp
     */
    struct tm timeinfo = {};
    timeinfo.tm_year = nmea_year + 100; // The year starts counting from 1900
    timeinfo.tm_mon = nmea_mth - 1; // The month starts counting from 0, so you need to subtract 1
    timeinfo.tm_mday = nmea_day;
    timeinfo.tm_hour = utc_hour;
    timeinfo.tm_min = utc_minute;
    timeinfo.tm_sec = int(utc_sec);
    timeinfo.tm_isdst = 0;

    time_t epoch = mktime(&timeinfo);

    if (epoch > GPS_EPOCH_SECS) {
        uint64_t usecs = static_cast<uint64_t>((utc_sec - static_cast<uint64_t>(utc_sec)) * 1000000);

        _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
        _gps_position->time_utc_usec += usecs;

    }
    else {
        _gps_position->time_utc_usec = 0;
    }

#else
    NMEA_UNUSED(utc_time);
    NMEA_UNUSED(nmea_date);
    _gps_position->time_utc_usec = 0;
#endif

    if (!_POS_received && (_last_POS_timeUTC < utc_time)) {
        _last_POS_timeUTC = utc_time;
        _POS_received = true;
    }

    if (!_VEL_received && (_last_VEL_timeUTC < utc_time)) {
        _last_VEL_timeUTC = utc_time;
        _VEL_received = true;
    }

    _TIME_received = true;

    return 1;
}

int
GPSDriverQL::decode_msg_gsa(char* bufptr)
{
    /*
    GPS DOP and active satellites
    An example of the GSA message string is:
    $GPGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>*<7><CR><LF>
    $GNGSA,A,3,82,67,74,68,73,80,83,,,,,,0.99,0.53,0.84,2*09
    $GNGSA,A,3,12,19,06,17,02,09,28,05,,,,,2.38,1.10,2.11,1*05
    $GNGSA,A,3,27,04,16,08,09,26,31,11,,,,,1.96,1.05,1.65,1*08

    GSA message fields
    Field	Meaning
    0	Message ID $GPGSA
    1	Mode 1, M = manual, A = automatic
    2	Mode 2, Fix type, 1 = not available, 2 = 2D, 3 = 3D
    3	PRN number, 01 through 32 for GPS, 33 through 64 for SBAS, 64+ for GLONASS
    4 	PDOP: 0.5 through 99.9
    5	HDOP: 0.5 through 99.9
    6	VDOP: 0.5 through 99.9
    7	The checksum data, always begins with *
    */
    char* endp;
    //char M_pos = ' ';
    int fix_mode = 0;
    uint8_t sat_id[12]{ 0 };
    //float pdop = 99.9f;
    float hdop = 99.9f, vdop = 99.9f;
    int sys_id = 0;
    int sat_count = 0;
    //uint8_t* used_svid = nullptr;

    if (bufptr && *(++bufptr) != ',') { /*M_pos = *(bufptr++);*/ }

    if (bufptr && *(++bufptr) != ',') { fix_mode = strtol(bufptr, &endp, 10); bufptr = endp; }

    for (int y = 0; y < 12; y++) {
        if (bufptr && *(++bufptr) != ',') { sat_count++; sat_id[y] = strtol(bufptr, &endp, 10); bufptr = endp; }
    }

    if (bufptr && *(++bufptr) != ',') { /*pdop = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { vdop = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { sys_id = strtod(bufptr, &endp); bufptr = endp; }

    if (fix_mode <= 1) {
        _gps_position->fix_type = 0;

    }
    else {
        _gps_position->hdop = static_cast<float>(hdop);
        _gps_position->vdop = static_cast<float>(vdop);
        _DOP_received = true;

        if (sys_id == GNSS_SYSTEM_ID_GPS) {
            memcpy(_gps_used_svid, sat_id, sat_count);
        }
        else if (sys_id == GNSS_SYSTEM_ID_GLONASS) {
            memcpy(_gln_used_svid, sat_id, sat_count);
        }
        else if (sys_id == GNSS_SYSTEM_ID_GALILEO) {
            memcpy(_gal_used_svid, sat_id, sat_count);
        }
        else if (sys_id == GNSS_SYSTEM_ID_BDS) {
            memcpy(_bds_used_svid, sat_id, sat_count);
        }
        else if (sys_id == GNSS_SYSTEM_ID_QZSS) {
            memcpy(_qzss_used_svid, sat_id, sat_count);
        }
        else {}
    }

    return 1;
}

int
GPSDriverQL::decode_msg_gsv(char* bufptr)
{
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
    char* endp;
    int all_page_num = 0, this_page_num = 0, tot_sv_visible = 0;
    uint8_t* used_svid = nullptr;
    uint8_t* sat_num_gsv = nullptr;
    uint8_t signal_id = 0;
    char* lastComma = nullptr;
    uint8_t init = 0;

    struct gsv_sat {
        int svid;
        int elevation;
        int azimuth;
        int snr;
    } sat[4]{};

    QL_DEBUG("Parsing GSV");

    if (bufptr && *(++bufptr) != ',') { all_page_num = strtol(bufptr, &endp, 10); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { this_page_num = strtol(bufptr, &endp, 10); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

    if ((this_page_num < 1) || (this_page_num > all_page_num)) {
        return 0;
    }
    lastComma = strrchr((char*)_rx_buffer, ',');
    if (lastComma && *(++lastComma) != ',') { signal_id = strtod(lastComma, &endp); }

    if (memcmp(_rx_buffer, "$GP", 3) == 0) {
        sat_num_gsv = &_sat_num_gpgsv;
        used_svid = _gps_used_svid;

    }
    else if (memcmp(_rx_buffer, "$GL", 3) == 0) {
        sat_num_gsv = &_sat_num_glgsv;
        used_svid = _gln_used_svid;
    }
    else if (memcmp(_rx_buffer, "$GA", 3) == 0) {
        sat_num_gsv = &_sat_num_gagsv;
        used_svid = _gal_used_svid;
    }
    else if (memcmp(_rx_buffer, "$GB", 3) == 0) {
        sat_num_gsv = &_sat_num_gbgsv;
        used_svid = _bds_used_svid;
    }
    else if (memcmp(_rx_buffer, "$GQ", 3) == 0) {
        sat_num_gsv = &_sat_num_gbgsv;
        used_svid = _bds_used_svid;
    }
    else {
        return 0;
    }

    if ((this_page_num == 1) && (signal_id == 1) && _satellite_info) {
        memset(_satellite_info->svid, 0, sizeof(_satellite_info->svid));
        memset(_satellite_info->used, 0, sizeof(_satellite_info->used));
        memset(_satellite_info->snr, 0, sizeof(_satellite_info->snr));
        memset(_satellite_info->elevation, 0, sizeof(_satellite_info->elevation));
        memset(_satellite_info->azimuth, 0, sizeof(_satellite_info->azimuth));
    }

    int end = 4;

    if (this_page_num == all_page_num) {
        end = tot_sv_visible - (this_page_num - 1) * 4;

        _SVNUM_received = true;
        _SVINFO_received = true;

        *sat_num_gsv += tot_sv_visible;

        if (_satellite_info) {
            _satellite_info->count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
            _satellite_info->timestamp = gps_absolute_time();
        }
    }
    if ((this_page_num == 1) && (signal_id != 1)) {
        for (int y = 0; y < satellite_info_s::SAT_INFO_MAX_SATELLITES; y++) {
            if (_satellite_info->svid[y] == 0) {
                break;
            }
            init++;
        }
    }

    if (_satellite_info) {
        int offset = 0;

        for (int y = 0; y < end; y++) {
            if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }

            if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }

            if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }

            if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

            offset = init + (y + (this_page_num - 1) * 4);
            if (offset > satellite_info_s::SAT_INFO_MAX_SATELLITES)
            {
                break;
            }

            _satellite_info->svid[offset] = sat[y].svid;
            _satellite_info->snr[offset] = sat[y].snr;
            _satellite_info->elevation[offset] = sat[y].elevation;
            _satellite_info->azimuth[offset] = sat[y].azimuth;

            if (is_used_svid(sat[y].svid, used_svid)) {
                _satellite_info->used[offset] = 1;
            }
            else {
                _satellite_info->used[offset] = 0;
            }
            //satellite_info->signal[offset] = signal_id;
        }
    }

    return 1;
}

int
GPSDriverQL::decode_msg_vtg(char* bufptr)
{
    /*$GNVTG,,T,,M,0.683,N,1.265,K*30
      $GNVTG,,T,,M,0.780,N,1.445,K*33

    Field	Meaning
    0	Message ID $GPVTG
    1	Track made good (degrees true)
    2	T: track made good is relative to true north
    3	Track made good (degrees magnetic)
    4	M: track made good is relative to magnetic north
    5	Speed, in knots
    6	N: speed is measured in knots
    7	Speed over ground in kilometers/hour (kph)
    8	K: speed over ground is measured in kph
    9	The checksum data, always begins with *
    */
    char* endp;
    float track_true = 0.f;
    //char T;
    //float track_mag = 0.f;
    //char M;
    float ground_speed = 0.f;
    //char N;
    //float ground_speed_K = 0.f;
    //char K;

    if (bufptr && *(++bufptr) != ',') { track_true = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { /*T = *(bufptr++);*/ }

    if (bufptr && *(++bufptr) != ',') { /*track_mag = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*M = *(bufptr++);*/ }

    if (bufptr && *(++bufptr) != ',') { ground_speed = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { /*N = *(bufptr++);*/ }

    if (bufptr && *(++bufptr) != ',') { /*ground_speed_K = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*K = *(bufptr++);*/ }

    float track_rad = track_true * M_PI_F / 180.0f; // rad in range [0, 2pi]

    if (track_rad > M_PI_F) {
        track_rad -= 2.f * M_PI_F; // rad in range [-pi, pi]
    }

    float velocity_ms = ground_speed / 1.9438445f;
    float velocity_north = velocity_ms * cosf(track_rad);
    float velocity_east = velocity_ms * sinf(track_rad);

    _gps_position->vel_m_s = velocity_ms;
    _gps_position->vel_n_m_s = velocity_north;
    _gps_position->vel_e_m_s = velocity_east;
    _gps_position->cog_rad = track_rad;
    _gps_position->vel_ned_valid = true; /** Flag to indicate if NED speed is valid */
    _gps_position->c_variance_rad = 0.1f;
    _gps_position->s_variance_m_s = 0;

    if (!_VEL_received) {
        _VEL_received = true;
    }

    if (_sat_num_gga > 0) {
        _gps_position->satellites_used = _sat_num_gga;

    }
    else if (_SVNUM_received && _SVINFO_received && _FIX_received) {

        _sat_num_gsv = _sat_num_gpgsv + _sat_num_glgsv + _sat_num_gagsv
            + _sat_num_gbgsv + _sat_num_gqgsv;
        _gps_position->satellites_used = MAX(_sat_num_gns, _sat_num_gsv);
    }

    if (_VEL_received && _POS_received) {
        _gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
        _rate_count_vel++;
        _rate_count_lat_lon++;
    }

    return 1;
}

int
GPSDriverQL::decode_msg_pqtmvel(char* bufptr)
{
    /*
    $PQTMVEL,1,051648.000,0.003,-0.006,-0.001,0.007,0.007,299.249,0.075,0.076,359.999*68

    Field	Meaning
    0	Message ID $PQTMVEL
    1	version
    2	UTC time
    3	North velocity (m/s)
    4	East velocity (m/s)
    5	Down velocity (m/s)
    6	2D speed (m/s)
    7	3D speed (m/s)
    8	Heading (Degree)
    9	Estimate of 2D speed accuracy (m/s)
    10  Estimate of 3D speed accuracy (m/s)
    11  Estimate of heading accuracy (Degree)
    */

    char* endp;
    //uint8_t version = 0;
    //float utc_time;
    double vel_n = 0, vel_e = 0, vel_d = 0;
    //double grd_spd = 0, spd = 0;
    double heading = 0;
    //double grd_spd_acc = 0, spd_acc = 0;
    double heading_acc = 0;

    if (bufptr && *(++bufptr) != ',') { /*version = strtol(bufptr, &endp, 10); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*utc_time = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { vel_n = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { vel_e = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { vel_d = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { /*grd_spd = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*spd = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { heading = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { /*grd_spd_acc = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*spd_acc = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { heading_acc = strtof(bufptr, &endp); bufptr = endp; }

    _gps_position->vel_n_m_s = vel_n;
    _gps_position->vel_e_m_s = vel_e;
    _gps_position->vel_d_m_s = vel_d;
    _gps_position->heading = heading;
    _gps_position->heading_accuracy = heading_acc;
    _gps_position->timestamp = gps_absolute_time();

    return 1;
}

int
GPSDriverQL::decode_msg_pqtmepe(char* bufptr)
{
    /*
    $PQTMEPE,2,1.6247,1.6247,4.0980,2.2977,4.6982*69

    Field	Meaning
    0	Message ID $PQTMEPE
    1	version
    2	Estimated north error.
    3	Estimated east error.
    4	Estimated down error.
    5	Estimated 2D position error.
    6	Estimated 3D position error.
    */

    char* endp;
    //uint8_t version = 0;
    //double epe_north = 0, epe_east = 0;
    double epe_down = 0;
    double epe_2d = 0;
    /*double epe_3d = 0;*/


    if (bufptr && *(++bufptr) != ',') { /*version = strtol(bufptr, &endp, 10); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*epe_north = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { /*epe_east = strtof(bufptr, &endp); bufptr = endp;*/ }

    if (bufptr && *(++bufptr) != ',') { epe_down = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { epe_2d = strtof(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { /*epe_3d = strtof(bufptr, &endp); bufptr = endp;*/ }

    _gps_position->eph = epe_2d;
    _gps_position->epv = epe_down;
    _gps_position->timestamp = gps_absolute_time();

    return 1;
}

int
GPSDriverQL::decode_msg_pairspf(char* bufptr)
{
    /*
    $PAIRSPF,0*53

    Field	Meaning
    0	Message ID $PAIRSPF
    1   Jamming status.
    */

    char* endp;
    uint8_t status = 0;


    if (bufptr && *(++bufptr) != ',') { status = strtol(bufptr, &endp, 10); bufptr = endp; }


    _gps_position->jamming_l1_state = status;

    _gps_position->timestamp = gps_absolute_time();

    return 1;
}

int
GPSDriverQL::decode_msg_pairspf5(char* bufptr)
{
    /*
    $PAIRSPF5,0*66

    Field	Meaning
    0	Message ID $PAIRSPF
    1   Jamming status.
    */

    char* endp;
    uint8_t status = 0;


    if (bufptr && *(++bufptr) != ',') { status = strtol(bufptr, &endp, 10); bufptr = endp; }


    _gps_position->jamming_l5_state = status;

    _gps_position->timestamp = gps_absolute_time();

    return 1;
}

// void
// GPSDriverQL::enc_gps_position_info_str(uint8_t* log)
// {
//     uint8_t buff[1024]{ 0 };

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "\r\n gps_position_info. \r\n");

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff),  "timestamp: %lu\r\n", _gps_position->timestamp);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "timestamp_sample: %lu\r\n", _gps_position->timestamp_sample);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "latitude_deg: %.3f\r\n", _gps_position->latitude_deg);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "longitude_deg: %.3f\r\n", _gps_position->longitude_deg);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "altitude_msl_m: %.3f\r\n", _gps_position->altitude_msl_m);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "altitude_ellipsoid_m: %.3f\r\n", _gps_position->altitude_ellipsoid_m);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "time_utc_usec: %lu\r\n", _gps_position->time_utc_usec);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "device_id: %d\r\n", _gps_position->device_id);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "s_variance_m_s: %.3f\r\n", _gps_position->s_variance_m_s);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "c_variance_rad: %.3f\r\n", _gps_position->c_variance_rad);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "eph: %.3f\r\n", _gps_position->eph);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "epv: %.3f\r\n", _gps_position->epv);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "hdop: %.3f\r\n", _gps_position->hdop);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "vdop: %.3f\r\n", _gps_position->vdop);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "noise_per_ms: %d\r\n", _gps_position->noise_per_ms);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "jamming_indicator: %d\r\n", _gps_position->jamming_indicator);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "vel_m_s: %.3f\r\n", _gps_position->vel_m_s);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "vel_n_m_s: %.3f\r\n", _gps_position->vel_n_m_s);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "vel_e_m_s:%.3f\r\n", _gps_position->vel_e_m_s);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "vel_d_m_s: %.3f\r\n", _gps_position->vel_d_m_s);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "cog_rad: %.3f\r\n", _gps_position->cog_rad);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "timestamp_time_relative: %d\r\n", _gps_position->timestamp_time_relative);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "heading: %.3f\r\n", _gps_position->heading);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "heading_offset: %.3f\r\n", _gps_position->heading_offset);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "heading_accuracy: %.3f\r\n", _gps_position->heading_accuracy);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "rtcm_injection_rate: %.3f\r\n", _gps_position->rtcm_injection_rate);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "automatic_gain_control: %d\r\n", _gps_position->automatic_gain_control);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "fix_type: %d\r\n", _gps_position->fix_type);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "jamming_state 1l: %d, l5: %d\r\n", _gps_position->jamming_l1_state, _gps_position->jamming_l5_state);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "vel_ned_valid: %d\r\n", _gps_position->vel_ned_valid);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "satellites_used: %d\r\n", _gps_position->satellites_used);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "selected_rtcm_instance: %d\r\n", _gps_position->selected_rtcm_instance);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "rtcm_crc_failed: %d\r\n", _gps_position->rtcm_crc_failed);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "rtcm_msg_used: %d\r\n", _gps_position->rtcm_msg_used);

//     snprintf((char*)buff + strlen((char*)buff), sizeof(buff), "_padding0: %d,%d \r\n\r\n", _gps_position->_padding0[0], _gps_position->_padding0[1]);

//     memcpy(log, buff, strlen((char*)buff) + 1);
// }
