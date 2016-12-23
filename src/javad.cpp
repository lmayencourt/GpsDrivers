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

#include <vector>

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
    GPS_INFO("JAVAD: ------- kill -------");
}

int GPSDriverJavad::configure(unsigned &baudrate, GPSHelper::OutputMode output_mode)
{
    GPS_INFO("JAVAD: ------- configure -------");
    if (output_mode != OutputMode::GPS) {
        GPS_WARN("JAVAD: Unsupported Output Mode %i", (int)output_mode);
        return -1;
    }

    // set baudrate
    if (GPSHelper::setBaudrate(JAVAD_BAUDRATE) != 0) {
        GPS_WARN("JAVAD: setBaudrate failed ");
        return -1;
    }

    baudrate = JAVAD_BAUDRATE;

    // send the configuration string to the JAVAD
    char initSequence[] = "%%dm,/dev/ser/a\n%%em,/dev/ser/a,jps{/RT,/PG,/VG,/SP,/SV,/DP,/XA,/SI,/EE}:{0.5}\n";
    if (write((void *) initSequence, strlen(initSequence)) == -1) {
        GPS_WARN("JAVAD: init sequence write error");
        return -1;
    }

    return 0;
}

// -1
int GPSDriverJavad::waitForAck(const unsigned timeout)
{
    int ret = -1;

    _ack_state = JAVAD_ACK_WAITING;

    gps_abstime time_started = gps_absolute_time();

    while ((_ack_state == JAVAD_ACK_WAITING) && (gps_absolute_time() < time_started + timeout * 1000)) {
        receive(timeout);
    }


    return ret;
}

// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
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
                if (_cbuffer.full() == false) {
                    if (_cbuffer.push(buf[i]) < 0)
                        // buffer full
                        return -1;
                }
                else
                    handled |= parse();
            }

            handled |= parse();

            if (handled > 0)
                return handled;

        } else if (ret == 0) {
            /* no data : return if message handled */
            GPS_WARN("JAVAD: nothing to read %d", handled);


            if (handled > 0)
                return handled;
        }

        /* in case we keep trying but only get crap from GPS */
        if (time_started + timeout * 1000 < gps_absolute_time()) {
            return -1;
        }
    }
}

// 0 = decoding, 1 = message handled
int GPSDriverJavad::parse()
{
    int msglen;
    int ret = 0;

    // check cbuffer occupancy
    while (_cbuffer.getOccupancy() > JN_MIN_MSG_LEN) {
//        GPS_INFO("JAVAD: buffer %d",_cbuffer.getOccupancy()) ;
        // get ID and length
        if (_cbuffer.at(0) != '\n') {
             _id.p[0] = _cbuffer.at(0);
             _id.p[1] = _cbuffer.at(1);

             if (checkID(_id) == true) {
                 _blength.p[3] = 0;
                 _blength.p[2] = _cbuffer.at(2);
                 _blength.p[1] = _cbuffer.at(3);
                 _blength.p[0] = _cbuffer.at(4);

                 msglen = hex2dec((char *) _blength.p, 4);

                 // check msg length
                 if (msglen > 0 && msglen < JN_MAX_DATA_LEN) {
                    msglen += JN_HEAD_LEN;

                    if (_cbuffer.getOccupancy() >= msglen) {
                        // whole message found

                        u1 checksum = cs(msglen -1);
                        u1 msg_checksum = _cbuffer.at(msglen -1);
                        // TMP: disable checksum
                        if (checksum == msg_checksum) {
                            // msg found
                            ret = handleMessage(msglen);
                            _cbuffer.popMultiple(msglen);

                            if (ret > 0)
                                return ret;

                        }
                        else {    // checksum doesn't match
//                            _debug_counter.bad_checksum_counter++;
                            _cbuffer.popMultiple(msglen);
                        }

                    }
                    else {
//                        _debug_counter.not_enough_data_counter++;
                        break;
                    }
                 }
                 else {    // bad length
//                     _debug_counter.bad_lenght_msg_counter++;
    //                 _cbuffer.pop();
                 }
             }
             else {     // not supported ID
//                 _debug_counter.bad_id_counter++;
                 _cbuffer.pop();
             }
        }
        else
            _cbuffer.pop();
    }

    return ret;
}

// 0 = message not supported, 1 = message handled
int GPSDriverJavad::handleMessage(int msglen)
{
    int ret = 0;
    int i, from = JN_HEAD_LEN;

//    GPS_INFO("JAVAD: handle msg %c%c",_id.p[0], _id.p[1]) ;

    switch (_id.n) {
    case JAVAD_ID_RT:
//        GPS_INFO("JAVAD: parse RT") ;
        for(i=0; i<4; i++) _epochTime.p[i]   = _cbuffer.at(from++);

        // New epoch
        _recvJNSMsg[RT] = 1;
        break;

    case JAVAD_ID_PV:
//        GPS_INFO("JAVAD: parse PV") ;
        for(i=0; i<8; i++) _pvMsg.x.p[i]    = _cbuffer.at(from++);
        for(i=0; i<8; i++) _pvMsg.y.p[i]    = _cbuffer.at(from++);
        for(i=0; i<8; i++) _pvMsg.z.p[i]    = _cbuffer.at(from++);
        for(i=0; i<4; i++) _pvMsg.psep.p[i] = _cbuffer.at(from++);
        for(i=0; i<4; i++) _pvMsg.vx.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _pvMsg.vy.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _pvMsg.vz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _pvMsg.vsep.p[i] = _cbuffer.at(from++);
        _pvMsg.type = _cbuffer.at(from);

        _recvJNSMsg[PV] = 1;
        break;

    case JAVAD_ID_PG:
//        GPS_INFO("JAVAD: parse PG") ;
        for(i=0; i<8; i++) _pgMsg.lat.p[i]    = _cbuffer.at(from++);
        for(i=0; i<8; i++) _pgMsg.lon.p[i]    = _cbuffer.at(from++);
        for(i=0; i<8; i++) _pgMsg.alt.p[i]    = _cbuffer.at(from++);
        for(i=0; i<4; i++) _pgMsg.psep.p[i] = _cbuffer.at(from++);
        _pgMsg.type = _cbuffer.at(from);

        _recvJNSMsg[PG] = 1;
        break;

    case JAVAD_ID_VG:
//        GPS_INFO("JAVAD: parse VG") ;
        for(i=0; i<4; i++) _vgMsg.vN.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _vgMsg.vE.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _vgMsg.vU.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _vgMsg.vsep.p[i] = _cbuffer.at(from++);
        _vgMsg.type = _cbuffer.at(from);

        _recvJNSMsg[VG] = 1;
        break;

    case JAVAD_ID_SP:
//        GPS_INFO("JAVAD: parse SP") ;
        for(i=0; i<4; i++) _spMsg.xx.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.yy.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.zz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.tt.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.xy.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.xz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.xt.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.yz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.yt.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _spMsg.zt.p[i]   = _cbuffer.at(from++);
        _spMsg.type = _cbuffer.at(from);

        _recvJNSMsg[SP] = 1;
        break;

    case JAVAD_ID_SV:
//        GPS_INFO("JAVAD: parse SV") ;
        for(i=0; i<4; i++) _svMsg.xx.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.yy.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.zz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.tt.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.xy.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.xz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.xt.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.yz.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.yt.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _svMsg.zt.p[i]   = _cbuffer.at(from++);
        _svMsg.type = _cbuffer.at(from);

        _recvJNSMsg[SV] = 1;
        break;

    case JAVAD_ID_DP:
//        GPS_INFO("JAVAD: parse DP") ;
        for(i=0; i<4; i++) _dpMsg.hdop.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _dpMsg.vdop.p[i]   = _cbuffer.at(from++);
        for(i=0; i<4; i++) _dpMsg.tdop.p[i]   = _cbuffer.at(from++);
        _dpMsg.solType = _cbuffer.at(from);

        _recvJNSMsg[DP] = 1;
        break;

    case JAVAD_ID_SI:
//        GPS_INFO("JAVAD: parse SI") ;
        _sat_count = msglen - 6;
        _siMsg.usi = new u1[_sat_count];

        for (i=0; i<_sat_count; i++)
            _siMsg.usi[i] = _cbuffer.at(from++);

        _recvJNSMsg[SI] = 1;
        break;

    case JAVAD_ID_RE:
        _
        break;

    case JAVAD_ID_EE:
        ret = handleEpoch();
        break;

    default :
//        GPS_INFO("JAVAD: handle not supported msg");
        break;
    }

    return ret;
}

int GPSDriverJavad::handleEpoch()
{
    int ret = 0;

    //    GPS_INFO("JAVAD: handle message...");

        if (_recvJNSMsg[RT] == 1)
        {
            if(_recvJNSMsg[PG] && _recvJNSMsg[VG] && _recvJNSMsg[SP] && _recvJNSMsg[SV] && _recvJNSMsg[DP])
            {
    //         GPS_INFO("JAVAD: get start of epoch");
            _gps_position->timestamp = gps_absolute_time();
            _last_timestamp_time = _gps_position->timestamp;

            // End of epoch => update
            GPS_INFO("JAVAD: get end of epoch > %d %d %d %d %d %d %d %d",
                                                _recvJNSMsg[RT],
                                                _recvJNSMsg[PG],
                                                _recvJNSMsg[VG],
                                                _recvJNSMsg[SP],
                                                _recvJNSMsg[SV],
                                                _recvJNSMsg[DP],
                                                _recvJNSMsg[PV],
                                                _recvJNSMsg[SI]);

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
                        if(_svMsg.xx.n > _svMsg.yy.n)
                            _gps_position->eph = sqrt(_svMsg.xx.n);
                        else
                            _gps_position->eph = sqrt(_svMsg.yy.n);

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
        //            _satellite_info->timestamp = gps_absolute_time();

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
//                    struct tm timeinfo;

        ////            timeinfo.tm_mday = packet.date / 10000;
        ////            timeinfo_conversion_temp = packet.date - timeinfo.tm_mday * 10000;
        ////            timeinfo.tm_mon = (timeinfo_conversion_temp / 100) - 1;
        ////            timeinfo.tm_year = (timeinfo_conversion_temp - (timeinfo.tm_mon + 1) * 100) + 100;


//                    uint32_t tmp = _epochTime.n / 1000;
//                    timeinfo.tm_hour = (uint32_t)(tmp / (60*60)) ;
//                    tmp = tmp % (60*60);
//                    timeinfo.tm_min = (uint32_t)(tmp / (60));
//                    tmp = tmp % 60;
//                    timeinfo.tm_sec = (uint32_t)(tmp);


    //                uint64_t usecs = ((uint32_t)(_epochTime.n) % 1000 ) * 1000ULL;
    //                timespec ts;
    //                ts.tv_sec = ;
    //                ts.tv_nsec = usecs;

    //                setClock(ts);

//        #ifndef NO_MKTIME

//                    time_t epoch = mktime(&timeinfo);

//                    if (epoch > GPS_EPOCH_SECS) {
//                        // FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
//                        // and control its drift. Since we rely on the HRT for our monotonic
//                        // clock, updating it from time to time is safe.

//                        uint64_t usecs = ((uint32_t)(_epochTime.n) % 1000 ) * 1000ULL;

//                        timespec ts;
//                        ts.tv_sec = epoch;
//                        ts.tv_nsec = usecs;

//                        setClock(ts);

//                        _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
//                        _gps_position->time_utc_usec += usecs;

//                    } else {
//                        _gps_position->time_utc_usec = 0;
//                    }

//        #else
//                    _gps_position->time_utc_usec = 0;
//        #endif

                _gps_position->timestamp = gps_absolute_time();
                _gps_position->timestamp_time_relative = 0;

                // end of epoch
                _recvJNSMsg[RT] = 0;
                _recvJNSMsg[PG] = 0;
                _recvJNSMsg[VG] = 0;
                _recvJNSMsg[SP] = 0;
                _recvJNSMsg[SV] = 0;
                _recvJNSMsg[DP] = 0;
                _recvJNSMsg[PV] = 0;
                _recvJNSMsg[SI] = 0;
            }

         }

    //    GPS_INFO("JAVAD: handle message end");
        return ret;

    }

void GPSDriverJavad::decodeInit()
{
    _cbuffer.empty();

    _recvJNSMsg[RT] = 0;
    _recvJNSMsg[PG] = 0;
    _recvJNSMsg[VG] = 0;
    _recvJNSMsg[SP] = 0;
    _recvJNSMsg[SV] = 0;
    _recvJNSMsg[DP] = 0;
    _recvJNSMsg[PV] = 0;
    _recvJNSMsg[SI] = 0;
}

// 0 = ID not supported, 1 = valid ID
bool GPSDriverJavad::checkID(type_msgID id)
{
    switch (id.n) {
    case JAVAD_ID_RT:
    case JAVAD_ID_PV:
    case JAVAD_ID_PG:
    case JAVAD_ID_VG:
    case JAVAD_ID_SP:
    case JAVAD_ID_SV:
    case JAVAD_ID_DP:
    case JAVAD_ID_SI:
    case JAVAD_ID_EE:
    case JAVAD_ID_RE:
        return true;
    default :
        return false;
    }

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
u1 GPSDriverJavad::cs(int count)
{
    enum { bits = 8, lShift = 2, rShift = bits - lShift	};

    int i = 0;
    u1 res = 0;

    while (count--) {
        res = ROT_LEFT(res) ^ _cbuffer.at(i);
        i++;
    }

    res = ROT_LEFT(res);

    return (res);
}

// ////////////////////////////////////////////////////////////////
//
// CircularBuffer implementation
//
// ////////////////////////////////////////////////////////////////
GPSDriverJavad::CircularBuffer::CircularBuffer()
{
    _head = 0;
    _tail = 0;
    _count = 0;
}

bool GPSDriverJavad::CircularBuffer::empty()
{
    _head = 0;
    _tail = 0;
    return (_count == 0);
}

bool GPSDriverJavad::CircularBuffer::full()
{
    return (_count == JAVAD_RECV_BUFFER_SIZE -1);
}

int GPSDriverJavad::CircularBuffer::getOccupancy()
{
    return _count;
}

int GPSDriverJavad::CircularBuffer::pop()
{
    uint8_t c;
    return pop(&c);
}

int GPSDriverJavad::CircularBuffer::popMultiple(int n)
{
    if (_count >= n) {
        if ((_tail + n) < JAVAD_RECV_BUFFER_SIZE)
            _tail += n;
        else
            _tail = _tail + n - JAVAD_RECV_BUFFER_SIZE;

        _count -= n;
    }
    else
        return -1;

    return 0;
}

int GPSDriverJavad::CircularBuffer::pop(uint8_t* c)
{
    if (_count > 0) {
        *c = _buffer[_tail];
        if (_tail == JAVAD_RECV_BUFFER_SIZE -1)
            _tail = 0;
        else
            _tail++;

        _count--;
    }
    else
        return -1;

    return 0;
}

int GPSDriverJavad::CircularBuffer::push(const uint8_t c)
{
    if (_count < JAVAD_RECV_BUFFER_SIZE ) {
        _buffer[_head] = c;
        if (_head == JAVAD_RECV_BUFFER_SIZE -1)
            _head = 0;
        else
            _head++;

        _count++;
    }
    else
        return -1;

    return 0;

}

uint8_t GPSDriverJavad::CircularBuffer::at(uint32_t p)
{
    if ((_tail + p) < JAVAD_RECV_BUFFER_SIZE)
        return _buffer[_tail + p];
    else
        return _buffer[_tail + p - JAVAD_RECV_BUFFER_SIZE];
}
