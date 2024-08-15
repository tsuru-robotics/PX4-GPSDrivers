#pragma once




#include "gps_helper.h"
#include "../../definitions.h"

#define QL_INFO(...) GPS_INFO(__VA_ARGS__)
#define QL_WARN(...) GPS_WARN(__VA_ARGS__)
#define QL_ERR(...)  GPS_ERR(__VA_ARGS__)

typedef enum {
	QL_DECODE_UNINIT = 0,
	QL_DECODE_GOT_SYNC,
	QL_DECODE_GOT_ASTERIK,
	QL_DECODE_GOT_FIRST_CS_BYTE,
	QL_DECODE_RTCM3
} ql_decode_state_t;


class GPSDriverQL : public GPSHelper
{
public:
	GPSDriverQL(GPSCallbackPtr callback, void* callback_user,
	sensor_gps_s* gps_position,
	satellite_info_s *satellite_info);
	virtual ~GPSDriverQL() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned& baudrate, const GPSConfig& config) override;

	// void enc_gps_position_info_str(uint8_t* log);
private:

#define QL_RX_BUFF_LENGTH  (1024)

	/**
	 * Parse the QL packet
	 */
	int parseChar(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	int handleMessage(int packet_len);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit();

	void msgFlagInit();

	bool is_used_svid(uint8_t svid, uint8_t* used_svid);
	bool is_same_nmea_msg_id(int offset, const char* msg_id);

	int decode_msg_gga(char* bufptr);
	int decode_msg_rmc(char* bufptr);
	int decode_msg_gsa(char* bufptr);
	int decode_msg_gsv(char* bufptr);
	int decode_msg_vtg(char* bufptr);
	int decode_msg_pqtmvel(char* bufptr);
	int decode_msg_pqtmepe(char* bufptr);
	int decode_msg_pairspf(char* bufptr);
	int decode_msg_pairspf5(char* bufptr);

	sensor_gps_s* _gps_position{ nullptr };
	satellite_info_s *_satellite_info {nullptr};

	uint8_t _gps_used_svid[satellite_info_s::SAT_INFO_MAX_SATELLITES]{ 0 };  // GPS
	uint8_t _gln_used_svid[satellite_info_s::SAT_INFO_MAX_SATELLITES]{ 0 };  // GLONASS
	uint8_t _bds_used_svid[satellite_info_s::SAT_INFO_MAX_SATELLITES]{ 0 };  // BDS
	uint8_t _gal_used_svid[satellite_info_s::SAT_INFO_MAX_SATELLITES]{ 0 };  // Galileo
	uint8_t _qzss_used_svid[satellite_info_s::SAT_INFO_MAX_SATELLITES]{ 0 };  // QZSS

	double _last_POS_timeUTC{ 0 };
	double _last_VEL_timeUTC{ 0 };
	double _last_FIX_timeUTC{ 0 };
	uint64_t _last_timestamp_time{ 0 };

	uint8_t _sat_num_gga{ 0 };
	uint8_t _sat_num_gns{ 0 };
	uint8_t _sat_num_gsv{ 0 };
	uint8_t _sat_num_gpgsv{ 0 };
	uint8_t _sat_num_glgsv{ 0 };
	uint8_t _sat_num_gagsv{ 0 };
	uint8_t _sat_num_gbgsv{ 0 };
	uint8_t _sat_num_gqgsv{ 0 };

	//  check if we got all basic essential packages we need
	bool _TIME_received{ false };
	bool _POS_received{ false };
	bool _ALT_received{ false };
	bool _SVNUM_received{ false };
	bool _SVINFO_received{ false };
	bool _FIX_received{ false };
	bool _DOP_received{ false };
	bool _VEL_received{ false };
	bool _EPH_received{ false };
	bool _HEAD_received{ false };

	bool _is_frame_end{ false };


//	RTCMParsing* _rtcm_parsing{ nullptr };
	ql_decode_state_t _decode_state{ QL_DECODE_UNINIT };
	unsigned _rx_count{};
	uint8_t _rx_buffer[QL_RX_BUFF_LENGTH];
	uint8_t _rx_ck_a{};
	uint8_t _rx_ck_b{};
};
