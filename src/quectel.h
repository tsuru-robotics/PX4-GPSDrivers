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
 * @file quectel.h
 *
 * Quectel protocol definitions
 *
 * @author Vladimir Savelyev <vms@flyfire.io>
 *
 */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

class RTCMParsing;

class GPSDriverQL : public GPSHelper
{
public:

	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverQL(GPSCallbackPtr callback, void *callback_user,
		      sensor_gps_s *gps_position,
		      satellite_info_s *satellite_info,
		      float heading_offset = 0.f);

	virtual ~GPSDriverQL();

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, const GPSConfig &config) override;

private:

	static constexpr unsigned QL_CONFIG_TIMEOUT = 500; // ms, timeout for waiting ACK
	static constexpr unsigned QL_OUT_MSG_MAX_SIZE = 50;
	static constexpr unsigned QL_RECV_BUFFER_SIZE = 1024;

	// NMEA messages
	enum class QlNmeaMsgId {
		GGA = 0,
		GLL,
		GSA,
		GSV,
		RMC,
		VTG,
		ZDA, // Not supported on LC29H (BA, CA, DA, EA)
		GRS, // Not supported on LC29H (BA, CA, DA, EA)
		GST  // Not supported on LC29H (BA, CA, DA, EA)
	};
	static constexpr unsigned QL_SET_NMEA_OUTPUT_RATE = 62;
	static constexpr unsigned QL_SET_DEBUGLOG_OUTPUT = 86;


	enum class QlPqtmMsgVer {
		NONE,
		VER1,
		VER2
	};

	void handleHeading(float heading_deg, float heading_stddev_deg);

	enum class NMEADecodeState {
		uninit,
		got_sync1,
		got_asteriks,
		got_first_cs_byte,
		decode_rtcm3
	};

	void decodeInit(void);
	int handleMessage(int len);
	int parseChar(uint8_t b);

	int32_t read_int();
	double read_float();
	char read_char();

	bool configMessages(const QlMsgConfig &config);

	bool setNmeaMsgOutputRate(QlNmeaMsgId nmea_msg_type, unsigned msg_rate);

	bool setNmeaDebugMode(unsigned mode);

	bool waitForNmeaAck(uint8_t command, unsigned timeout);

	bool setPqtmMsgOutputRate(const char pqtm_msg_name[], unsigned msg_rate, QlPqtmMsgVer msg_ver);

	bool setPqtmDebugMode(unsigned mode);

	bool waitForPqtmAck(char msg[QL_OUT_MSG_MAX_SIZE], unsigned timeout);

	int calcChecksum(const char *msg, size_t msg_length, char* checksum);

	bool writeMessage(char msg[QL_OUT_MSG_MAX_SIZE]);

	sensor_gps_s *_gps_position {nullptr};
	satellite_info_s *_satellite_info {nullptr};
	uint64_t _last_timestamp_time{0};

	//  check if we got all basic essential packages we need
	bool _waiting_for_ACK{false};
	bool _ACK_received{false};
	bool _POS_received{false};
	bool _VEL_received{false};
	bool _SVINFO_received{false};

	NMEADecodeState _decode_state{NMEADecodeState::uninit};
	uint8_t _rx_buffer[QL_RECV_BUFFER_SIZE] {};
	uint16_t _rx_buffer_bytes{0};

	bool _ack_nmea_command{false}; // true - ACK
	uint8_t _ack_nmea_command_id{0};
	uint8_t _ack_nmea_command_error_code{0};
	bool _ack_pqtm_command{false}; // true - ACK
	uint8_t _ack_pqtm_command_error_code{0};

	OutputMode _output_mode{OutputMode::GPS};

	RTCMParsing *_rtcm_parsing{nullptr};

	float _epe_multiplier{1.0f}; // multiplier for eph/epv from EPE message

};
