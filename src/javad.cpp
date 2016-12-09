/****************************************************************************
 *
 *   Copyright (C) 2016. All rights reserved.
 *   Author: Mayencourt Louis <louis.mayencourt@epfl.ch>
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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctime>

#include "javad.h"

#define ROT_LEFT(val) ((val << lShift) | (val >> rShift))

GPSDriverJavad::GPSDriverJavad(GPSCallbackPtr callback, void *callback_user,
                               struct vehicle_gps_position_s *gps_position,
                               struct satellite_info_s *satellite_info):
      GPSHelper(callback, callback_user),
      _gps_position(gps_position),
      _satellite_info(satellite_info)
{
    decodeInit();
}

GPSDriverJavad::~GPSDriverJavad()
{

}

int GPSDriverJavad::configure(unsigned &baudrate, GPSHelper::OutputMode output_mode)
{
    if (output_mode != OutputMode::GPS) {
        GPS_WARN("JAVAD: Unsupported Output Mode %i", (int)output_mode);
        return -1;
    }

    // set baudrate
    if (GPSHelper::setBaudrate(JAVAD_BAUDRATE) != 0) {
        return -1;
    }

    baudrate = JAVAD_BAUDRATE;

    // send the configuration string to the JAVAD
//    char initSequence[] = "%%dm,/dev/ser/a\n%%em,/dev/ser/a,jps{/RT,/PG,/VG,/SP,/SV,/DP,/XA,/SI,/EE}:{0.1}";
//    if (write((void *) initSequence, strlen(initSequence)) == -1) {
//        GPS_WARN("JAVAD: init sequence write error");
//        return -1;
//    }

    return 0;
}

// // -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
int GPSDriverJavad::receive(unsigned timeout)
{
    uint8_t buf[GPS_READ_BUFFER_SIZE];

//    GPS_INFO("JAVAD: receive...");

    /* timeout additional to poll */
    gps_abstime time_started = gps_absolute_time();

    int handled = 0;

    while (true) {

        int ret = read(buf, sizeof(buf), timeout);

        if (ret < 0) {
            /* something went wrong when polling or reading */
//            GPS_WARN("JAVAD: read return < 0");
            return -1;

        } else if (ret > 0) {
//            GPS_INFO("JAVAD: copy %d to rx buffer", ret);
            /* pass received bytes to the packet decoder */
            for (int i = 0; i < ret; i++) {
//                handled = parseChar(buf[i]);
                addChar(buf[i]);
            }

            handled = handleMessage();

        } else if (ret == 0) {
            /* no data : return if message handled */

            if (handled > 0) {
//                GPS_WARN("JAVAD: return %d", handled);
                return handled;
            }
        }

        /* in case we keep trying but only get crap from GPS */
        if (time_started + timeout * 1000 < gps_absolute_time()) {
            GPS_WARN("JAVAD: timeout");
            return -1;
        }
    }
}

// 0 = decoding, 1 = message handled
int GPSDriverJavad::parseChar(const uint8_t b)
{
    int ret = 0;

    switch (_decode_state) {

    /* Expecting start of EPOC ~~ (RT) 1*/
    case JAVAD_DECODE_SYNCH1:
        if (b == JAVAD_ID_RT0) {
            _decode_state = JAVAD_DECODE_SYNCH2;
            _rx_buffer[_rx_buffer_len++] = b;
        }
        break;

    /* Expecting start of EPOC ~~ (RT) 2*/
    case JAVAD_DECODE_SYNCH2:
        if (b == JAVAD_ID_RT0) {
            _decode_state = JAVAD_DECODE_PAYLOAD;
            _rx_buffer[_rx_buffer_len++] = b;
        }
        break;

    case JAVAD_DECODE_PAYLOAD:
        _rx_buffer[_rx_buffer_len++] = b;

        if (b == JAVAD_ID_EE0) { // end of EPOC detected
            _decode_state = JAVAD_DECODE_END1;
        }
        break;

    /* Expecting end of EPOC || (EE) */
    case JAVAD_DECODE_END1:
        if (b == JAVAD_ID_EE1) {
            _rx_buffer[_rx_buffer_len++] = b;

            ret = handleMessage();
            decodeInit();
        }
        break;

    default:
        break;
    }

    return ret;
}

// 0 = message not found, 1 = message handled, 2 = sat info and pos
int GPSDriverJavad::handleMessage()
{
    int ret = 0;
    double RT_epoch;

//    GPS_INFO("JAVAD: handle message...");

    // [~~] JAVAD Message
    // First message of ensemble of messages of an epoch
    if (getRT(RT_epoch))
    {
//         GPS_INFO("JAVAD: get start of epoch");
        _gps_position->timestamp = gps_absolute_time();
        _last_timestamp_time = _gps_position->timestamp;

        // New epoch
        _recvJNSMsg[RT] = 1;
        _recvJNSMsg[PG] = 0;
        _recvJNSMsg[VG] = 0;
        _recvJNSMsg[SP] = 0;
        _recvJNSMsg[SV] = 0;
        _recvJNSMsg[DP] = 0;
        _recvJNSMsg[PV] = 0;
        _recvJNSMsg[SI] = 0;
    }

    if (_recvJNSMsg[RT] == 1)
    {
        if (_recvJNSMsg[PG] == 0)
        {
            if (getPG(_pgMsg)) {
                _recvJNSMsg[PG] = 1;
//                GPS_INFO("JAVAD: get PG");
            }
        }

        if (_recvJNSMsg[VG] == 0)
        {
            if (getVG(_vgMsg)) {
                _recvJNSMsg[VG] = 1;
//                GPS_INFO("JAVAD: get VG");
            }
        }

        if (_recvJNSMsg[SP] == 0)
        {
            if (getSP(_spMsg)) {
                _recvJNSMsg[SP] = 1;
//                GPS_INFO("JAVAD: get SP");
            }
        }

        if (_recvJNSMsg[SV] == 0)
            {
            if (getSV(_svMsg)) {
                _recvJNSMsg[SV] = 1;
//                GPS_INFO("JAVAD: get SV");
            }
        }

        if (_recvJNSMsg[DP] == 0)
            {
            if (getDP(_dpMsg)) {
                _recvJNSMsg[DP] = 1;
//                GPS_INFO("JAVAD: get DP");
            }
        }

        if (_recvJNSMsg[PV] == 0)
            {
            if (getPV(_pvMsg)) {
                _recvJNSMsg[PV] = 1;
//                GPS_INFO("JAVAD: get PV");
            }
        }

        if (_recvJNSMsg[SI] == 0)
        {
            if (getSI(_siMsg, _sat_count)) {
                _recvJNSMsg[SI] = 1;
//                GPS_INFO("JAVAD: get SI");
            }
        }

        if (getEE()) {
            // End of epoch => update
//            GPS_INFO("JAVAD: get end of epoch");

            // by default fix_type no fix
            _gps_position->fix_type = 0;

            if (_recvJNSMsg[PG]) {
                double tmp;

                tmp = _pgMsg.lat.n * 180.0 / (double)(M_PI_F);
                _gps_position->lat = (int32_t)(tmp * 1E7);
                tmp = _pgMsg.lon.n * 180.0 / (double)(M_PI_F);
                _gps_position->lon = (int32_t)(tmp * 1E7);
                tmp = _pgMsg.alt.n * 180.0 / (double)(M_PI_F);
                _gps_position->alt_ellipsoid = (int32_t)(tmp * 1E3);

//                GPS_INFO("JAVAD: gps lat > %d", _gps_position->lat);

                switch (_pgMsg.type) {
                case 1: _gps_position->fix_type = 3; break;
                case 2: _gps_position->fix_type = 4; break;
                case 3: _gps_position->fix_type = 5; break;
                case 4: _gps_position->fix_type = 6; break;
                case 5: _gps_position->fix_type = 0; break;
                default: _gps_position->fix_type = 0;
                    break;
                }

                _rate_count_lat_lon++;
            }

            if (_recvJNSMsg[SP]) {
                // max of xx or yy
                _gps_position->eph = sqrt(_svMsg.xx.n);

                _gps_position->epv = sqrt(_svMsg.zz.n);
            }

            if (_recvJNSMsg[SV]) {
                // to correct after
                _gps_position->s_variance_m_s = sqrt(_svMsg.xx.n + _svMsg.yy.n +
                                                     _svMsg.zz.n + _svMsg.tt.n);
            }

            if (_recvJNSMsg[VG]) {
                _gps_position->vel_n_m_s = _vgMsg.vN.n;
                _gps_position->vel_e_m_s = _vgMsg.vE.n;
                _gps_position->vel_d_m_s = -_vgMsg.vU.n;
                _gps_position->vel_m_s = sqrt(_vgMsg.vN.n * _vgMsg.vN.n +
                                              _vgMsg.vE.n * _vgMsg.vE.n);
                _gps_position->cog_rad = atan2(_vgMsg.vE.n, _vgMsg.vN.n);

                _rate_count_vel++;
            }

            if (_recvJNSMsg[DP]) {
                _gps_position->hdop = _dpMsg.hdop.n;
                _gps_position->vdop = _dpMsg.vdop.n;
 //                _gps_position->fix_type = _pgMsg.type + 2;
            }

//            _gps_position->c_variance_rad;
//            _gps_position->noise_per_ms;
//            _gps_position->jamming_indicator;
            _gps_position->vel_ned_valid = true;

            if (_recvJNSMsg[PG] && _recvJNSMsg[VG])
                ret = 1;

            if (_recvJNSMsg[SI]) {
                _satellite_info->timestamp = gps_absolute_time();

                _gps_position->satellites_used = _sat_count;
                _satellite_info->count = _sat_count;

                for (int i=0; i<_sat_count; i++) {
                    _satellite_info->svid[i] = _siMsg.usi[i];
                    _satellite_info->used[i] = true;
                    _satellite_info->snr[i]	= 1;
                    _satellite_info->elevation[i] = 1;
                    _satellite_info->azimuth[i]	= 1;
                }                
                ret = 3;
            }

//            /* convert time and date information to unix timestamp */
//            struct tm timeinfo;

////            timeinfo.tm_mday = packet.date / 10000;
////            timeinfo_conversion_temp = packet.date - timeinfo.tm_mday * 10000;
////            timeinfo.tm_mon = (timeinfo_conversion_temp / 100) - 1;
////            timeinfo.tm_year = (timeinfo_conversion_temp - (timeinfo.tm_mon + 1) * 100) + 100;

//            GPS_INFO("JAVAD: epoch > %d", RT_epoch);

//            timeinfo.tm_hour = (uint32_t)(RT_epoch / 1000*60*60) % 24;
//            timeinfo.tm_min = (uint32_t)(RT_epoch / 1000*60) % 60;
//            timeinfo.tm_sec = (uint32_t)(RT_epoch / 1000) % 60;

//            GPS_INFO("JAVAD: time > %dh %dm %ds", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

//#ifndef NO_MKTIME

//            time_t epoch = mktime(&timeinfo);

//            if (epoch > GPS_EPOCH_SECS) {
//                // FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
//                // and control its drift. Since we rely on the HRT for our monotonic
//                // clock, updating it from time to time is safe.

//                    GPS_INFO("JAVAD: call mktime");
//                uint64_t usecs = ((uint32_t)(RT_epoch) % 1000 ) * 1000ULL;

//                timespec ts;
//                ts.tv_sec = epoch;
//                ts.tv_nsec = usecs;

//                setClock(ts);

//                _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
//                _gps_position->time_utc_usec += usecs;

//            } else {
//                _gps_position->time_utc_usec = 0;
//            }

//#else
//            _gps_position->time_utc_usec = 0;
//#endif

            _gps_position->timestamp = gps_absolute_time();
            _gps_position->timestamp_time_relative = 0;



            // end of epoch
            _recvJNSMsg[RT] = 0;
        }
     }

//    GPS_INFO("JAVAD: handle message end");
    return ret;

}

void GPSDriverJavad::decodeInit()
{
    _decode_state = JAVAD_DECODE_SYNCH1;
    _rx_buffer_len = 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse RT message from the input buffer
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getRT(double &time)
{
    unsigned char c1='~';
    unsigned char c2='~';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        long ltime;
        memcpy(&ltime, &_rx_buffer[start + JN_HEAD_LEN], 4);
        time = (double) (ltime/1000.0);

        removeMsgFromBuf(0, start + len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse PV message from the input buffer, failes = 0;
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getPV(SMsgPV &pv)
{
    unsigned char c1='P';
    unsigned char c2='V';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        int i, from = start + JN_HEAD_LEN;
        for(i=0; i<8; i++) pv.x.p[i]    = _rx_buffer[from++];
        for(i=0; i<8; i++) pv.y.p[i]    = _rx_buffer[from++];
        for(i=0; i<8; i++) pv.z.p[i]    = _rx_buffer[from++];
        for(i=0; i<4; i++) pv.psep.p[i] = _rx_buffer[from++];
        for(i=0; i<4; i++) pv.vx.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) pv.vy.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) pv.vz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) pv.vsep.p[i] = _rx_buffer[from++];
        pv.type = _rx_buffer[from];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}


// ////////////////////////////////////////////////////////////////
//
// parse PG message from the input buffer, failes = 0;
//
// ////////////////////////////////////////////////////////////////

int GPSDriverJavad::getPG (SMsgPG &pg)
{
    unsigned char c1='P';
    unsigned char c2='G';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        int i, from = start + JN_HEAD_LEN;
        for(i=0; i<8; i++) pg.lat.p[i]    = _rx_buffer[from++];
        for(i=0; i<8; i++) pg.lon.p[i]    = _rx_buffer[from++];
        for(i=0; i<8; i++) pg.alt.p[i]    = _rx_buffer[from++];
        for(i=0; i<4; i++) pg.psep.p[i] = _rx_buffer[from++];
        pg.type = _rx_buffer[from];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse PV message from the input buffer, failes = 0;
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getVG (SMsgVG &vg)
{
    unsigned char c1='V';
    unsigned char c2='G';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        int i, from = start + JN_HEAD_LEN;
        for(i=0; i<4; i++) vg.vN.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) vg.vE.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) vg.vU.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) vg.vsep.p[i] = _rx_buffer[from++];
        vg.type = _rx_buffer[from];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse SP message from the input buffer, failes = 0;
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getSP(SMsgSP &sp)
{
    unsigned char c1='S';
    unsigned char c2='V';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        int i, from = start + JN_HEAD_LEN;
        for(i=0; i<4; i++) sp.xx.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.yy.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.zz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.tt.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.xy.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.xz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.xt.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.yz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.yt.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sp.zt.p[i]   = _rx_buffer[from++];
        sp.type = _rx_buffer[from];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse SV message from the input buffer, failes = 0;
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getSV(SMsgSV &sv)
{
    unsigned char c1='S';
    unsigned char c2='V';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        int i, from = start + JN_HEAD_LEN;
        for(i=0; i<4; i++) sv.xx.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.yy.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.zz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.tt.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.xy.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.xz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.xt.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.yz.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.yt.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) sv.zt.p[i]   = _rx_buffer[from++];
        sv.type = _rx_buffer[from];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}


// ////////////////////////////////////////////////////////////////
//
// parse DP message from the input buffer, failes = 0;
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getDP(SMsgDP &dp)
{
    unsigned char c1='D';
    unsigned char c2='P';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        int i, from = start + JN_HEAD_LEN;
        for(i=0; i<4; i++) dp.hdop.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) dp.vdop.p[i]   = _rx_buffer[from++];
        for(i=0; i<4; i++) dp.tdop.p[i]   = _rx_buffer[from++];
        dp.solType = _rx_buffer[from];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse SI message from the input buffer, failes = 0;
// must delete array usi after calling this method
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getSI(SMsgSI &si, int &nrSat)
{
    unsigned char c1='S';
    unsigned char c2='I';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    nrSat = 0;

    // msg found, checksum OK
    if (answ == 4)
    {
        nrSat = len - 6;
        si.usi = new u1[nrSat];

        int from = start + JN_HEAD_LEN;
        for (int i=0; i<nrSat; i++)
            si.usi[i] = _rx_buffer[from++];

        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// parse EE (epoch end) message from an input buffer
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::getEE()
{
    unsigned char c1='|';
    unsigned char c2='|';
    unsigned char MsgId[2] = {c1, c2};

    int start=0, len=0;
    int answ = findMsg(MsgId, start, len);

    // msg found, checksum OK
    if (answ == 4)
    {
        removeMsgFromBuf(start, len);

        return 1;
    }

    // msg is corrupted, checksum doesn't match
    if (answ == 3)
        removeMsgFromBuf(start, len);

    return 0;
}

// ////////////////////////////////////////////////////////////////
//
// FIND message in _rx_buffer
//
// return if
//     1 - message body identifier found &&
//     2 - second msg lenght found &&
//     3 - message body found && (but check sum not OK!)
//     4 - check sum OK
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::findMsg(unsigned char *MsgID, int &msgstart, int &msglen)
{
    int idfound=0, mlfound=0, mbfound=0, csfound=0;
    int i=0, lpos=0;

SEARCH_AGAIN:

    for (; i<_rx_buffer_len - 1; i++)
    {
        if (_rx_buffer[i] == MsgID[0] && _rx_buffer[i+1] == MsgID[1])
        {
            idfound = 1;					// ID found
            lpos    = i;
            break;
        }
    }

    if (idfound)
    {
        if (_rx_buffer_len > lpos + 4)
        {
            mlfound = 1;					// ID + msglen found!

            id[0] = _rx_buffer[lpos];
            id[1] = _rx_buffer[lpos + 1];

            blength.p[3] = 0;
            blength.p[2] = _rx_buffer[lpos + 2];
            blength.p[1] = _rx_buffer[lpos + 3];
            blength.p[0] = _rx_buffer[lpos + 4];

            msglen = hex2dec((char *) blength.p, 4);

            // hal.console->printf("%c%c l:%d", id[0],id[1], msglen);

            if (msglen > 0 && msglen < 4096)
            {
                msglen  += JN_HEAD_LEN;

                if (_rx_buffer_len > lpos + msglen)
                {
                    mbfound  = 1;				// whole message found
                    msgstart = lpos;

                    u1 checksum = cs(&_rx_buffer[msgstart], (msglen - 1));
                    // TMP: disable checksum
                    if (checksum == _rx_buffer[msgstart + msglen - 1])
                        csfound = 1;			// checksum is OK

                    // hal.console->printf("c: %d vs %d; ", checksum,_rx_buffer[msgstart + msglen - 1]);
                }
            }

            // Not a valid message. Search again
            else
            {
                i       = lpos + 2;
                lpos    = i;
                idfound = 0;
                mlfound = 0;
                mbfound = 0;
                csfound = 0;
                goto SEARCH_AGAIN;
            }
        }
    }

    return (idfound + mlfound + mbfound + csfound);
}

// ////////////////////////////////////////////////////////////////
//
// place char in buff
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::addChar(u1 mychar)
{
    if (_rx_buffer_len < JN_MAX_DATA_LEN) // add it at the buf end
        _rx_buffer[_rx_buffer_len++] = mychar;
    else
    {
        GPS_WARN("JAVAD: rx buffer full !");
        memcpy(&_rx_buffer[0], &_rx_buffer[1], _rx_buffer_len - 1);
        _rx_buffer[_rx_buffer_len-1] = mychar;
    }

    return _rx_buffer_len;
}

// ////////////////////////////////////////////////////////////////
//
// int setBufSize(int len)
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::setBufSize(int len)
{
    _rx_buffer_len = len < JN_MAX_DATA_LEN ? len : JN_MAX_DATA_LEN;
    return _rx_buffer_len;
}

// ////////////////////////////////////////////////////////////////
//
// remove message from the buf
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::removeMsgFromBuf(int start, int len)
{
//    GPS_INFO("JAVAD: remove %d", len);
    int offset = start + len;

    if (offset < _rx_buffer_len)
    {
        while (offset < _rx_buffer_len)
            _rx_buffer[start++] = _rx_buffer[offset++];
    }

    return (setBufSize(start));
}

// ////////////////////////////////////////////////////////////////
//
// convert hex to dec
//
// ////////////////////////////////////////////////////////////////
int GPSDriverJavad::hex2dec(char *hex, int len)
{
    int dec = 0, pw = 1;

    for(int i=0; i<len; i++)
    {
        int c = hex[i];

        if (c >= '0' && c <= '9')
            c -= '0';
        else if (c >= 'A' && c <= 'F')
            c -= ('A' - 10);

        dec += c * pw;
        pw *= 16;

    }

    return (dec);
}

// ////////////////////////////////////////////////////////////////
// message check sum, Javad code from Grill
//
// For messages, 8-bit checksum is computed starting with the first byte of the
// message identifier and ending with the byte immediately preceding the checksum
// field57.
//
// ////////////////////////////////////////////////////////////////
u1 GPSDriverJavad::cs(u1 const* buf, int count)
{
    enum { bits = 8, lShift = 2, rShift = bits - lShift	};

    u1 res = 0;

    while (count--)
        res = ROT_LEFT(res) ^ *buf++;

    res = ROT_LEFT(res);

    return (res);
}
