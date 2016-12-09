/****************************************************************************
 *
 *   Copyright (C) 2016. All rights reserved.
 *   Author: Mayencourt Louis <louis.mayencourt@epfl.ch>
 *
 *   JNS Implemented by Jan Skaloud 01-2004
 *   Extention by Jan Skaloud 11-2004: addition of messages GT, XA,
 *   Modified by Phillip Tom� 01-2006: addition of AddString method to CJNSMsg)
 *	 Modified by Phillip Tom� 01-2006: addition of messages SI, EL, AZ
 *   Modified by Phillip Tom� 02-2006: modification to the arguments and return
 *                                     value of several functions
 *   Modified by Phillip Tom� 04-2006: definition of JN_HEAD_LEN
 *   Modified by Phillip Tom� 01-2008: addition of message SG
 *   Extention by Yannick Stebler	02-2009: addition of ASCII message extraction
 *	 Extention by Yannick Stebler	02-2009: addition of messages EC,1P,2P,DP,CP,RC,RM
 *   Ported to px4 by Louis Mayencourt 11-2017
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
 * @file javad.h
 *
 * Javad protocol definition.
 *
 * @author Mayencourt Louis <louis.mayencourt@epfl.ch>
 *
 */

#ifndef JAVAD_H_
#define JAVAD_H_

#include "gps_helper.h"
#include "../../definitions.h"

#define JAVAD_BAUDRATE 57600
#define JAVAD_RECV_BUFFER_SIZE 1024

// HEADER LENGTH
#define JN_HEAD_LEN		5

// MAXIMUM MESSAGE LENGTH (BYTES)
#define JN_MAX_DATA_LEN 512

/* Message IDs */
#define JAVAD_ID_RT0    '~'
#define JAVAD_ID_RT1    '~'
#define JAVAD_ID_EE0    '|'
#define JAVAD_ID_EE1    '|'

///* General: Header */
//typedef struct {
//    uint8_t		id1;
//    uint8_t		id2;
//    uint8_t     lenght1;
//    uint8_t     lenght2;
//    uint8_t     lenght3;
//    uint8_t*    msg;
//    uint16_t	chksum;
//} javad_header_t;

/* JNS DATA TYPES */
typedef char					a1;									// 1-byte ASCII character
typedef int8_t					i1;									// 1-byte signed integer
typedef int16_t					i2;									// 2-byte signed integer
typedef int32_t					i4 __attribute__((__may_alias__));	// 4-byte signed integer
typedef uint8_t					u1;									// 1-byte unsigned integer
typedef uint16_t				u2;									// 2-byte unsigned integer
typedef uint32_t				u4;									// 4-byte unsigned integer
typedef float					f4 __attribute__((__may_alias__));	// 4-byte floating point (IEEE) number
//typedef softfloat::sffloat64 	f8;									// 8-byte floating point (IEEE) number
typedef double                  f8;
typedef unsigned char	 		*str_t;								// str zero-terminated sequence of ASCII characters
typedef unsigned short			nstr_t;								// sequence of ASCII characters prepended by an u1-type
                                    // value specifying the number of characters in the sequence
//////////////////////////////////////////////////////////////
//    IMPLEMENTED JNS MESSAGE TYPES
//////////////////////////////////////////////////////////////

enum MsgType{ RT, PV, PG, VG, SP, SV, DP, SI, EE, count }; // implemented messages

typedef union { u1 p[4]; u4 n; } type_msglen;

// [~~] Receiver Time {5}
struct SMsgRT
{
    u4 tod;					// Tr modulo 1 day (86400000 ms) [ms]
    u1 cs;					// Checksum
};

// [PV] Cartesian Position and Velocity {46}
//		f8 x, y, z;			// Cartesian coordinates [m]
//		f4 psep;			// position SEP [m]
//		f4 vx, vy, vz;		// Cartesian velocities [m/s]
//		f4 vSigma;			// velocity SEP [m/s]
typedef	union { u1 p[8]; f8 n; } type_xyz;
typedef	union { u1 p[4]; f4 n; } type_vel;
typedef union { u1 p[4]; f4 n; } type_sep;
struct SMsgPV
{
    type_xyz x;
    type_xyz y;
    type_xyz z;
    type_sep psep;
    type_vel vx;
    type_vel vy;
    type_vel vz;
    type_sep vsep;
    u1 type;				// enum SolutionType
    u1 cs;					// Checksum
};

// [PG] Geodetic Position
//		f8 lat;			// Latitude [rad]
//		f8 lon;			// Longitude [rad]
//		f8 alt;			// Ellipsoidal height [m]
//		f4 pSigma;		// Position SEP [m]
//		u1 solType;		// Solution type
struct SMsgPG
{
    type_xyz lat;
    type_xyz lon;
    type_xyz alt;
    type_sep psep;
    u1 type;				// enum SolutionType
    u1 cs;					// Checksum
};

// [VG] Geodetic Velocity
// 		f4 lat;			// Northing velocity [m/s]
// 		f4 lon;			// Easting velocity [m/s]
// 		f4 alt;			// Height velocity [m/s]
// 		f4 pSigma;		// Velocity SEP [m/s]
// 		u1 solType;		// Solution type
struct SMsgVG
{
    type_vel vN;
    type_vel vE;
    type_vel vU;
    type_sep vsep;
    u1 type;				// enum SolutionType
    u1 cs;					// Checksum
};

// [SP] Poistion Covariance Matrix
// 		f4 xx;			// [m^2]
// 		f4 yy;			// [m^2]
// 		f4 zz;			// [m^2]
// 		f4 tt;          // [m^2]
// 		f4 xy;			// [m^2]
// 		f4 xz;			// [m^2]
// 		f4 xt;			// [m^2]
// 		f4 yz;          // [m^2]
// 		f4 yt;			// [m^2]
// 		f4 zt;			// [m^2]
// 		u1 solType;		// Solution type
struct SMsgSP
{
    type_vel xx;
    type_vel yy;
    type_vel zz;
    type_vel tt;
    type_vel xy;
    type_vel xz;
    type_vel xt;
    type_vel yz;
    type_vel yt;
    type_vel zt;
    u1 type;				// enum SolutionType
    u1 cs;					// Checksum
};

// [SV] Velocity Covariance Matrix
// 		f4 xx;			// [(m/s)^2]
// 		f4 yy;			// [(m/s)^2]
// 		f4 zz;			// [(m/s)^2]
// 		f4 tt;          // [(m/s)^2]
// 		f4 xy;			// [(m/s)^2]
// 		f4 xz;			// [(m/s)^2]
// 		f4 xt;			// [(m/s)^2]
// 		f4 yz;          // [(m/s)^2]
// 		f4 yt;			// [(m/s)^2]
// 		f4 zt;			// [(m/s)^2]
// 		u1 solType;		// Solution type
struct SMsgSV
{
    type_vel xx;
    type_vel yy;
    type_vel zz;
    type_vel tt;
    type_vel xy;
    type_vel xz;
    type_vel xt;
    type_vel yz;
    type_vel yt;
    type_vel zt;
    u1 type;				// enum SolutionType
    u1 cs;					// Checksum
};

// [DP] Dilution Of Precision (DOP)
//
//
// f4 hdop;					// Horizontal dilution of precision (HDOP) []
// f4 vdop;					// Vertical dilution of precision (VDOP) []
// f4 tdop;					// Time dilution of precision (TDOP) []
// u1 solType;				// Solution type
// u1 cs;					// Checksum
typedef union { u1 p[4]; f4 n; } type_dop;

struct SMsgDP
{
    type_dop hdop;
    type_dop vdop;
    type_dop tdop;
    u1 solType;
    u1 cs;
};

// [SI] Satellite Indices Message {nSats+1}
//
//
//	u1 usi[nsats];			// satellites indices
//	u1 cs;					// checksum

struct SMsgSI
{
    SMsgSI()     { usi = NULL; }
    ~SMsgSI()    { Clean(); }
    void Clean() { if (usi != NULL) delete[] usi; usi = NULL; }

    u1 *usi;				// satellites indices
    u1 cs;					// checksum
};

/* Decoder state */
typedef enum {
    JAVAD_DECODE_SYNCH1 = 0,
    JAVAD_DECODE_SYNCH2,
    JAVAD_DECODE_PAYLOAD,
    JAVAD_DECODE_END1
} javad_decode_state_t;

class GPSDriverJavad : public GPSHelper
{
public :
    GPSDriverJavad(GPSCallbackPtr callback, void *callback_user,
             struct vehicle_gps_position_s *gps_position,
             struct satellite_info_s *satellite_info);
    virtual ~GPSDriverJavad();

    int configure(unsigned &baudrate, OutputMode output_mode);
    int receive(unsigned timeout);

private:

    /**
     * Parse the binary JAVAD packet
     */
    int parseChar(const uint8_t b);

    /**
     * Handle the package once it has arrived
     */
    int handleMessage();

    /**
     * Reset the parse state machine for a fresh start
     */
    void decodeInit();

    int getRT(double &time);
    int getPV(SMsgPV &pv);
    int getPG(SMsgPG &pg);
    int getVG(SMsgVG &vg);
    int getSP(SMsgSP &sp);
    int getSV(SMsgSV &sv);
    int getDP(SMsgDP &dp);
    int getSI(SMsgSI &si, int &nrSat);
    int getEE();

    int findMsg(unsigned char *MsgID, int &msgstart, int &msglen);
    int addChar(u1 mychar);
    int setBufSize(int len);
    int removeMsgFromBuf(int start, int len);

    int hex2dec(char *hex, int len);
    u1 cs(u1 const* buf, int count);

    struct vehicle_gps_position_s *_gps_position;
    struct satellite_info_s *_satellite_info;

    int _sat_count;
    uint64_t _last_timestamp_time;
    javad_decode_state_t _decode_state;
    uint8_t _rx_buffer[JAVAD_RECV_BUFFER_SIZE];
    uint32_t _rx_buffer_len;
    uint8_t _recvJNSMsg[MsgType::count];

    a1 id[2];						// message ID
    type_msglen blength;			// only [3] !!! // message body length - descriptor

    SMsgPV	_pvMsg;
    SMsgPG	_pgMsg;
    SMsgVG	_vgMsg;
    SMsgSP  _spMsg;
    SMsgSV  _svMsg;
    SMsgDP  _dpMsg;
    SMsgSI  _siMsg;
};

#endif /* JAVAD_H_ */
