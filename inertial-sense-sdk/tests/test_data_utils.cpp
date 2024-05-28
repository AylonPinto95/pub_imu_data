/**
 * @file test_data_utils.cpp
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Walt Johnson on 3/6/24
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include <chrono>
#include <iostream>
#include <random>

#include "ISFileManager.h"
#include "protocol_nmea.h"
#include "test_data_utils.h"
#include "time_conversion.h"

#define MIN_VALUE 		-1000
#define MAX_VALUE 		 1000

#if defined(_WIN32)
#define DATA_DIR ""
#elif defined(__GNUC__)
#define DATA_DIR "./"
#endif

#define PRINT_DEBUG 0
#if PRINT_DEBUG
#define DEBUG_PRINT(...)    printf("L%d: ", __LINE__); printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) 
#endif

using namespace std;

static const int s_maxFileSize = 5242880;
//static const int s_maxFileSize = 100000;	// Make many small files
static const float s_maxDiskSpacePercent = 0.5f;
static const float s_maxDiskSpaceMBLarge = 1024.0f * 1024.0f * 10.0f;
static const bool s_useTimestampSubFolder = false;
static uint32_t s_timeMs = 0;
static uint32_t s_gpsTowOffsetMs = 0;
static uint32_t s_gpsWeek = 0;
static double s_towOffset = 0;
static const uint32_t s_timePeriodMs = 10;
static const uint32_t s_pimuPeriodMs = 10;
static const uint32_t s_navPeriodMs = 100;
static const uint32_t s_gpsPeriodMs = 200;

static pimu_t       s_pimu = {};
static ins_1_t      s_ins1 = {};
static gps_pos_t    s_gpsPos = {};
static gps_vel_t    s_gpsVel = {};

struct sTimeMs
{
    uint32_t pimu;
    uint32_t ins1;
    uint32_t gpsPos;
    uint32_t gpsVel;

    uint32_t nmeaPImu;
    uint32_t nmeaIns1;
    uint32_t nmeaGpsPos;
    uint32_t nmeaZda;
    uint32_t nmeaGga;

    uint32_t ubxNav;
    uint32_t ubxRxm;

    uint32_t rtcm1005;
    uint32_t rtcm1007;
    uint32_t rtcm1033;
    uint32_t rtcm1085;
    uint32_t rtcm1095;
} s_msgTimeMs = {};

void CurrentGpsTimeMs(uint32_t &gpsTimeOfWeekMs, uint32_t &gpsWeek)
{
    // Get current time in UTC
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    const std::tm *utc_tm = std::gmtime(&now_c);

    // Convert current UTC time to GPS time
    stdUtcDateTimeToGpsTime(*utc_tm, C_GPS_LEAP_SECONDS, gpsTimeOfWeekMs, gpsWeek);
    gpsTimeOfWeekMs *= 1000;
}

void PrintUtcTime(std::tm &utcTime, uint32_t milliseconds)
{
    printf( "UTC Time: %04d-%02d-%02d %02d:%02d:%02d.%03d\n", 
        utcTime.tm_year + 1900,   // tm_year is year since 1900
        utcTime.tm_mon + 1,       // tm_mon is months since January (0-11)
        utcTime.tm_mday,
        utcTime.tm_hour,
        utcTime.tm_min,
        utcTime.tm_sec,
        milliseconds );
}

bool periodCheck(uint32_t &msgTimeMs, uint32_t periodMs)
{
    if (s_timeMs < msgTimeMs)
    {   // Not time yet
        return false;
    }
    msgTimeMs = (s_timeMs - (s_timeMs%periodMs)) + periodMs;
    return true;
}

bool GeneratePimu(test_message_t &msg, pimu_t &pimu, int i, float f, bool init=false)
{
    if (init)
    {
        memset(&pimu, 0, sizeof(pimu_t));
        pimu.time = 0.001*((double)s_timeMs);
        return false;
    }

    if (!periodCheck(s_msgTimeMs.pimu, s_pimuPeriodMs))
    {   // Not time yet
        return false;
    }

    pimu.time = 0.001*((double)s_timeMs);
    pimu.status = i;
    pimu.dt = 0.004f;
    pimu.theta[0] += f*0.1f;
    pimu.theta[1] += f*0.2f;
    pimu.theta[2] += f*0.3f;
    pimu.vel[0] += f*0.4f;
    pimu.vel[1] += f*0.5f;
    pimu.vel[2] += f*0.6f;

    msg.data.pImu = pimu;
    msg.dataHdr.id = DID_PIMU;
    msg.dataHdr.size = sizeof(pimu_t);
    return true;
}

bool GenerateIns1(test_message_t &msg, ins_1_t &ins1, int i, float f, bool init=false)
{
    if (init)
    {
        memset(&ins1, 0, sizeof(ins_1_t));
        ins1.timeOfWeek = 0.001*((double)(s_timeMs + s_gpsTowOffsetMs));
        ins1.week = s_gpsWeek;
        ins1.lla[0] =   40.330565516;
        ins1.lla[1] = -111.725787806;
        ins1.lla[2] = 1408.565264;
        return false;
    }

    if (!periodCheck(s_msgTimeMs.ins1, s_navPeriodMs))
    {   // Not time yet
        return false;
    }

    ins1.timeOfWeek = 0.001*((double)(s_timeMs + s_gpsTowOffsetMs));
    ins1.week = s_gpsWeek;
    ins1.insStatus = i;
    ins1.hdwStatus = i;
    ins1.theta[0] += f*0.1f;
    ins1.theta[1] += f*0.2f;
    ins1.theta[2] += f*0.3f;
    ins1.uvw[0] += f*0.4f;
    ins1.uvw[1] += f*0.5f;
    ins1.uvw[2] += f*0.6f;
    ins1.lla[0] += f*0.001f;
    ins1.lla[1] += f*0.001f;
    ins1.lla[2] += f*0.001f;
    ins1.ned[0] += f*1.234f;
    ins1.ned[1] += f*2.345f;
    ins1.ned[2] += f*3.456f;

    msg.data.ins1 = ins1;
    msg.dataHdr.id = DID_INS_1;
    msg.dataHdr.size = sizeof(ins_1_t);
    return true;
}

bool GenerateGpsPos(test_message_t &msg, gps_pos_t &gps, int i, float f, bool init=false)
{
    if (init)
    {
        memset(&gps, 0, sizeof(gps_pos_t));
        gps.timeOfWeekMs = s_timeMs + s_gpsTowOffsetMs;
        gps.week = s_gpsWeek;
        gps.ecef[0] = f*123.4;
        gps.ecef[1] = f*234.5;
        gps.ecef[2] = f*345.6;
        gps.lla[0] =   40.330565516;
        gps.lla[1] = -111.725787806;
        gps.lla[2] = 1408.565264;
        gps.hAcc = (float)i;
        gps.cnoMean = (float)i;
        gps.hMSL = (float)i;
        gps.pDop = (float)i;
        gps.towOffset = (double)i*123.4;
        gps.leapS = C_GPS_LEAP_SECONDS;
        return false;
    }

    if (!periodCheck(s_msgTimeMs.gpsPos, s_gpsPeriodMs))
    {   // Not time yet
        return false;
    }

    gps.timeOfWeekMs = s_timeMs + s_gpsTowOffsetMs;
    gps.week = s_gpsWeek;
    gps.status = i;
    gps.ecef[0] = f*1.234;
    gps.ecef[1] = f*2.345;
    gps.ecef[2] = f*3.456;
    gps.lla[0] += f*0.001f;
    gps.lla[1] += f*0.001f;
    gps.lla[2] += f*0.001f;
    gps.hAcc = fabsf(f);
    gps.cnoMean = fabsf(f);
    gps.hMSL = fabsf(f);
    gps.pDop = fabsf(f);
    gps.towOffset = f;
    gps.leapS = C_GPS_LEAP_SECONDS;

    msg.data.gpsPos = gps;
    msg.dataHdr.id = DID_GPS1_POS;
    msg.dataHdr.size = sizeof(gps_pos_t);
    return true;
}

bool GenerateGpsVel(test_message_t &msg, gps_vel_t &gps, int i, float f, bool init=false)
{
    if (init)
    {
        memset(&gps, 0, sizeof(gps_vel_t));
        gps.timeOfWeekMs = s_timeMs + s_gpsTowOffsetMs;
        return false;
    }

    if (!periodCheck(s_msgTimeMs.gpsVel, s_gpsPeriodMs))
    {   // Not time yet
        return false;
    }

    gps.timeOfWeekMs = s_timeMs + s_gpsTowOffsetMs;
    gps.status = i;
    gps.vel[0] = f*12.34f;
    gps.vel[1] = f*23.45f;
    gps.vel[2] = f*34.56f;
    gps.sAcc = fabsf(f);

    msg.data.gpsVel = gps;
    msg.dataHdr.id = DID_GPS1_VEL;
    msg.dataHdr.size = sizeof(gps_vel_t);
    return true;
}

bool GenerateISB(test_message_t &msg, int i, float f)
{
    static bool init = true;

    msg.ptype = _PTYPE_INERTIAL_SENSE_DATA;

    // Initialize data
    if (init)
    {
        init = false;
        
        CurrentGpsTimeMs(s_gpsTowOffsetMs, s_gpsWeek);
        s_timeMs = 0;
        GeneratePimu(  msg, s_pimu,   i, f, true);
        GenerateIns1(  msg, s_ins1,   i, f, true);
        GenerateGpsPos(msg, s_gpsPos, i, f, true);
        GenerateGpsVel(msg, s_gpsVel, i, f, true);
    }

    if (GeneratePimu(  msg, s_pimu,   i, f)) { return true; }
    if (GenerateIns1(  msg, s_ins1,   i, f)) { return true; }
    if (GenerateGpsPos(msg, s_gpsPos, i, f)) { return true; }
    if (GenerateGpsVel(msg, s_gpsVel, i, f)) { return true; }

    return false;
}

bool timeIsSameAndSet(uint32_t &msgMs, uint32_t timeMs)
{
    if (msgMs != timeMs)
    {   
        msgMs = timeMs;
        return true;
    }

    return false;
}

bool GenerateNMEA(test_message_t &msg, int i, float f)
{
    if (timeIsSameAndSet(s_msgTimeMs.nmeaPImu, s_msgTimeMs.pimu))
    {
        msg.pktSize = nmea_ppimu((char*)msg.comm.rxBuf.start, msg.comm.rxBuf.size, s_pimu);
        msg.ptype = _PTYPE_NMEA;
        return true;
    }

    if (timeIsSameAndSet(s_msgTimeMs.nmeaZda, s_msgTimeMs.gpsPos))
    {   
        msg.pktSize = nmea_zda((char*)msg.comm.rxBuf.start, msg.comm.rxBuf.size, s_gpsPos);
        msg.ptype = _PTYPE_NMEA;
        printf("NMEA: %.*s", msg.pktSize, msg.comm.rxBuf.start);
        return true;
    }

    if (timeIsSameAndSet(s_msgTimeMs.nmeaGga, s_msgTimeMs.gpsPos))
    {   
        msg.pktSize = nmea_gga((char*)msg.comm.rxBuf.start, msg.comm.rxBuf.size, s_gpsPos);
        msg.ptype = _PTYPE_NMEA;
        return true;
    }

    if (timeIsSameAndSet(s_msgTimeMs.nmeaGpsPos, s_msgTimeMs.gpsPos))
    {
        msg.pktSize = nmea_pgpsp((char*)msg.comm.rxBuf.start, msg.comm.rxBuf.size, s_gpsPos, s_gpsVel);
        msg.ptype = _PTYPE_NMEA;
        return true;
    }

    if (timeIsSameAndSet(s_msgTimeMs.nmeaIns1, s_msgTimeMs.ins1))
    {
        msg.pktSize = nmea_pins1((char*)msg.comm.rxBuf.start, msg.comm.rxBuf.size, s_ins1);
        msg.ptype = _PTYPE_NMEA;
        return true;
    }

    return false;
}

bool GenerateUblox(test_message_t &msg, int i, float f)
{
    (void)f;

    if (periodCheck(s_msgTimeMs.ubxNav, s_gpsPeriodMs))
    {
        static int index = 0;

        if ((index++)%2)
        {   // Ublox - UBX-NAV-POSLLH (0x01 0x02)
            uint8_t buf[] = { 0xb5,0x62,0x1,0x2,0x1c,0x0,0x0,0xa1,0xad,0x10,0x6a,0xff,0x67,0xbd,0xb7,0xf4,0x9,0x18,0x35,0x7e,0x15,0x0,0xe8,0xc5,0x15,0x0,0x4f,0x1,0x0,0x0,0xa8,0x1,0x0,0x0,0x59,0xbc };
            msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);            
        }
        else
        {   // Ublox - UBX-NAV-POSLLH (0x01 0x02)
            uint8_t buf[] = { 0xb5,0x62,0x1,0x2,0x1c,0x0,0x90,0xa2,0xad,0x10,0x6a,0xff,0x67,0xbd,0xb7,0xf4,0x9,0x18,0x35,0x7e,0x15,0x0,0xdf,0xc5,0x15,0x0,0x4f,0x1,0x0,0x0,0xa8,0x1,0x0,0x0,0xe1,0x2b };
            msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        }
        msg.ptype = _PTYPE_UBLOX;
        return true;
    }

    if (periodCheck(s_msgTimeMs.ubxRxm, s_gpsPeriodMs))
    {   
        static int index = 0;
        if ((index++)%2)
        {   // Ublox - UBX-RXM-RAWX (0x02 0x15)
            uint8_t buf[] = { 0xb5,0x62,0x2,0x15,0xb0,0x3,0x1b,0x2f,0xdd,0x24,0x1b,0x14,0x11,0x41,0x2b,0x8,0x12,0x1d,0x1,0x1,0x82,0x9,0x5d,0x61,0x9b,0x3f,0xab,0xb,0x80,0x41,0x60,0x1a,0xa8,0x8a,0x7c,0x14,0xa5,0x41,0xa8,0x6e,0x9c,0x43,0x1,0x85,0x0,0x0,0xf4,0xfb,0x2c,0x5,0x1,0x8,0xf,0x0,0x39,0xe2,0x7d,0x36,0x68,0xc,0x83,0x41,0x45,0xc6,0xb1,0x7,0x6c,0x6,0xa9,0x41,0xa0,0xe6,0xc,0x44,0x5,0x2,0x0,0x0,0xf4,0xfb,0x25,0x6,0x1,0x8,0x7,0x0,0x56,0x5c,0xa2,0x55,0x34,0x9f,0x70,0x41,0x24,0xe2,0x8b,0xa8,0x50,0xd6,0x95,0x41,0x34,0x1d,0x31,0x45,0x0,0x5,0x0,0x0,0xf4,0xfb,0x30,0x5,0x1,0x8,0x7,0x0,0x1a,0xaa,0xe6,0x47,0x4,0xa7,0x73,0x41,0x5c,0x99,0x57,0xbc,0x8b,0xd1,0x99,0x41,0x96,0xcd,0x25,0xc5,0x0,0x13,0x0,0x0,0xf4,0xfb,0x26,0x5,0x1,0x8,0x7,0x0,0x79,0x9,0xfb,0xf3,0x4a,0x11,0x70,0x41,0x8e,0x98,0x96,0x32,0xe0,0x1b,0x95,0x41,0xb2,0x6,0xcf,0x44,0x0,0x19,0x0,0x0,0xf4,0xfb,0x2c,0x5,0x1,0x8,0x7,0x0,0x28,0x7c,0xed,0xa,0xb2,0xc3,0x73,0x41,0x7e,0x1f,0x51,0xa7,0x37,0xf7,0x99,0x41,0x16,0x48,0x95,0x44,0x2,0x1,0x0,0x0,0xf4,0xfb,0x29,0x5,0x1,0x8,0x7,0x0,0xe6,0x7a,0x56,0x4c,0x6f,0x33,0x6f,0x41,0xbe,0xd9,0x7f,0x82,0x8c,0xd3,0x94,0x41,0x20,0x25,0x5d,0x44,0x6,0x9,0x0,0x5,0xf4,0xfb,0x26,0x5,0x1,0x8,0x7,0x0,0xa9,0x32,0x13,0x58,0xfd,0x3e,0x6e,0x41,0xa7,0x9f,0x32,0xb6,0x77,0x39,0x94,0x41,0x7c,0x44,0x12,0x44,0x6,0x13,0x0,0xa,0xf4,0xfb,0x21,0x7,0x2,0x8,0x7,0x0,0xb,0x37,0x40,0xe6,0x53,0x54,0x74,0x41,0x50,0xb1,0xc8,0x15,0x3b,0xb5,0x9a,0x41,0xeb,0x3f,0x11,0x45,0x2,0x8,0x0,0x0,0xf4,0xfb,0x27,0x5,0x1,0x8,0x7,0x0,0x62,0xb8,0x14,0xb2,0x1a,0xd,0x72,0x41,0x6e,0xb2,0xbc,0x21,0x4,0xb7,0x97,0x41,0x5e,0x39,0xa5,0x44,0x2,0x1a,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0x7,0x0,0xa2,0xe1,0xa9,0xf7,0x62,0xe1,0x70,0x41,0x44,0x73,0xc5,0xd4,0x2d,0x97,0x96,0x41,0x1,0xff,0xf4,0x44,0x6,0x3,0x0,0xc,0xf4,0xfb,0x2b,0x5,0x1,0x8,0x7,0x0,0x18,0x57,0x42,0xb8,0x58,0x5e,0x73,0x41,0x68,0x95,0x61,0xc0,0x11,0x72,0x99,0x41,0xc2,0xe6,0xcb,0xc4,0x2,0x15,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0x7,0x0,0x43,0x99,0x17,0x5c,0xb5,0x22,0x72,0x41,0x8,0xc6,0x9b,0x64,0x5f,0x47,0x98,0x41,0x1e,0x2e,0x86,0x45,0x6,0x4,0x0,0xd,0xf4,0xfb,0x26,0x6,0x1,0x8,0x7,0x0,0x20,0xf5,0x6a,0x72,0xd9,0x46,0x77,0x41,0x42,0xe2,0x94,0x2e,0x7d,0x94,0x9e,0x41,0xd5,0xbb,0x3f,0x45,0x2,0x1f,0x0,0x0,0xf4,0xfb,0x23,0x6,0x1,0x8,0x7,0x0,0x74,0x1,0x2f,0x4d,0x45,0x5f,0x71,0x41,0x2c,0xa5,0xec,0x21,0xb2,0x28,0x97,0x41,0x5b,0x67,0x27,0x45,0x6,0xff,0x0,0x1,0xc0,0x12,0x27,0x5,0x1,0x8,0xf,0x0,0x7,0x49,0x2b,0x5d,0x78,0x3d,0x76,0x41,0x8,0xff,0xb7,0x8e,0xd8,0x37,0x9d,0x41,0xa4,0xc2,0xfa,0xc4,0x2,0xf,0x0,0x0,0xf4,0xfb,0x25,0x5,0x1,0x8,0x7,0x0,0x37,0x34,0x4e,0x78,0xb4,0xe,0x72,0x41,0xe5,0x50,0xde,0xfc,0x1e,0xb9,0x97,0x41,0x33,0xd4,0x35,0x45,0x0,0x1d,0x0,0x0,0xf4,0xfb,0x29,0x5,0x1,0x8,0x7,0x0,0x7b,0x1e,0x56,0x93,0xc0,0x37,0x72,0x41,0x95,0x61,0x9f,0xee,0xa,0xef,0x97,0x41,0xec,0x17,0x2a,0xc5,0x0,0x6,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0xf,0x0,0xe4,0x3e,0x36,0xf9,0x6,0xdf,0x7f,0x41,0xf5,0x4a,0xe7,0xa1,0x7e,0xef,0xa4,0x41,0x20,0xe0,0x9a,0x43,0x1,0x8a,0x0,0x0,0xf4,0xfb,0x2b,0x5,0x1,0x8,0xf,0x0,0xd,0xbd,0xcc,0xd2,0xc6,0xcd,0x72,0x41,0x9c,0x48,0x6a,0x1c,0x24,0xb4,0x98,0x41,0xd8,0xdd,0x5e,0xc4,0x2,0xd,0x0,0x0,0xf4,0xfb,0x28,0x5,0x1,0x8,0x7,0x0,0xaa,0x1b,0x81,0x57,0xb1,0x3f,0x74,0x41,0x4c,0xe0,0x4e,0x1e,0x1e,0x9a,0x9a,0x41,0xa9,0x3b,0x25,0xc5,0x0,0x18,0x0,0x0,0xf4,0xfb,0x24,0x6,0x1,0x8,0xf,0x0,0x22,0x41,0x5e,0xea,0xaf,0x9f,0x70,0x41,0xcb,0x9f,0xa7,0x62,0x45,0x33,0x96,0x41,0xe0,0xe9,0x5a,0xc5,0x6,0x10,0x0,0x6,0xf4,0xfb,0x24,0x6,0x1,0x8,0xf,0x0,0xce,0x25,0x25,0xa7,0xcf,0x6f,0x70,0x41,0x4e,0x82,0x95,0x5c,0xc,0x98,0x95,0x41,0x8e,0x1c,0x91,0xc4,0x0,0x2,0x0,0x0,0xf4,0xfb,0x2c,0x5,0x1,0x8,0x7,0x0,0x15,0x97,0x78,0xd7,0x1c,0x13,0x72,0x41,0x93,0xcc,0x58,0x82,0xc9,0x1c,0x98,0x41,0xae,0x75,0xae,0xc4,0x6,0x2,0x0,0x3,0xf4,0xfb,0x2a,0x5,0x1,0x8,0xf,0x0,0x71,0x2d,0xf8,0xb7,0xdd,0x23,0x72,0x41,0xd,0xbb,0xcd,0x6b,0x34,0x40,0x98,0x41,0xe1,0xa1,0x38,0x45,0x6,0x14,0x0,0x9,0xf4,0xfb,0x1e,0x8,0x3,0x8,0xf,0x0,0xba,0x40,0xb0,0xd2,0x7d,0x56,0x76,0x41,0xd7,0x44,0xd7,0xff,0xb7,0x58,0x9d,0x41,0x16,0x2f,0x35,0xc5,0x2,0x1b,0x0,0x0,0xf4,0xfb,0x24,0x5,0x1,0x8,0x7,0x0,0x36,0xb9,0xcd,0x0,0x91,0x19,0x74,0x41,0xf2,0xd,0x57,0x62,0x8,0x68,0x9a,0x41,0x8c,0x29,0xb7,0x44,0x0,0x1f,0x0,0x0,0xf4,0xfb,0x29,0x5,0x1,0x8,0xf,0x0,0xfe,0x54,0xb5,0xd5,0x99,0x2e,0x70,0x41,0xa0,0x27,0x9e,0x1c,0x5e,0x98,0x95,0x41,0x7,0x51,0xe5,0xc4,0x6,0x12,0x0,0x4,0xf4,0xfb,0x2c,0x5,0x1,0x8,0xf,0x0,0xd7,0x76,0x9,0xb8,0xcd,0xe7,0x6e,0x41,0x92,0x38,0xc7,0xe,0x19,0x4d,0x94,0x41,0x50,0x98,0x5a,0xc4,0x0,0xc,0x0,0x0,0xf4,0xfb,0x30,0x5,0x1,0x8,0xf,0x0,0xa7,0xd5 };
            msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        }
        else
        {   // Ublox - UBX-RXM-RAWX (0x02 0x15)
            uint8_t buf[] = { 0xb5,0x62,0x2,0x15,0xb0,0x3,0x81,0x95,0x43,0x8b,0x19,0x14,0x11,0x41,0x2b,0x8,0x12,0x1d,0x1,0x1,0xf2,0x7,0x6f,0x1f,0x77,0xfd,0xab,0xb,0x80,0x41,0xa0,0xcc,0x14,0x85,0x7d,0x14,0xa5,0x41,0x78,0x85,0x9c,0x43,0x1,0x85,0x0,0x0,0xf4,0xfb,0x2c,0x5,0x1,0x8,0xf,0x0,0x17,0xbd,0xc5,0x8d,0x69,0xc,0x83,0x41,0x12,0x42,0x8c,0xca,0x6d,0x6,0xa9,0x41,0x44,0xfc,0xc,0x44,0x5,0x2,0x0,0x0,0xf4,0xfb,0x25,0x6,0x1,0x8,0x7,0x0,0xae,0xc2,0xb4,0xd1,0x41,0x9f,0x70,0x41,0x2c,0xb7,0xc2,0x5e,0x62,0xd6,0x95,0x41,0x4e,0x22,0x31,0x45,0x0,0x5,0x0,0x0,0xf4,0xfb,0x30,0x5,0x1,0x8,0x7,0x0,0x94,0x60,0x66,0xa8,0xf7,0xa6,0x73,0x41,0x9d,0xff,0x92,0x27,0x7b,0xd1,0x99,0x41,0x8f,0xca,0x25,0xc5,0x0,0x13,0x0,0x0,0xf4,0xfb,0x27,0x5,0x1,0x8,0x7,0x0,0xb4,0xf1,0x8c,0xd3,0x52,0x11,0x70,0x41,0xc0,0x58,0x83,0x8c,0xea,0x1b,0x95,0x41,0x18,0xf,0xcf,0x44,0x0,0x19,0x0,0x0,0xf4,0xfb,0x2c,0x5,0x1,0x8,0x7,0x0,0x51,0x66,0xdb,0xb9,0xb7,0xc3,0x73,0x41,0x96,0x38,0x72,0x1e,0x3f,0xf7,0x99,0x41,0xb0,0x5c,0x95,0x44,0x2,0x1,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0x7,0x0,0x4c,0x65,0xc0,0x9a,0x77,0x33,0x6f,0x41,0xf0,0xbc,0x3d,0xa,0x92,0xd3,0x94,0x41,0x11,0x44,0x5d,0x44,0x6,0x9,0x0,0x5,0xf4,0xfb,0x26,0x5,0x1,0x8,0x7,0x0,0x0,0x9c,0xa6,0xcc,0x2,0x3f,0x6e,0x41,0x36,0x84,0x72,0x5f,0x7b,0x39,0x94,0x41,0x79,0x69,0x12,0x44,0x6,0x13,0x0,0xa,0xf4,0xfb,0x21,0x7,0x2,0x8,0x7,0x0,0xca,0x12,0x6,0xf2,0x5e,0x54,0x74,0x41,0x96,0x34,0xdd,0x9b,0x49,0xb5,0x9a,0x41,0x86,0x3e,0x11,0x45,0x2,0x8,0x0,0x0,0xf4,0xfb,0x27,0x5,0x1,0x8,0x7,0x0,0x47,0x7c,0x92,0xfa,0x20,0xd,0x72,0x41,0x92,0xd2,0x89,0x64,0xc,0xb7,0x97,0x41,0x6c,0x36,0xa5,0x44,0x2,0x1a,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0x7,0x0,0x8a,0x3e,0xbd,0x1e,0x6c,0xe1,0x70,0x41,0x3e,0x74,0x9b,0x14,0x3a,0x97,0x96,0x41,0x14,0x9,0xf5,0x44,0x6,0x3,0x0,0xc,0xf4,0xfb,0x2b,0x5,0x1,0x8,0x7,0x0,0x55,0xf2,0xb9,0xf5,0x50,0x5e,0x73,0x41,0x62,0x2b,0x75,0x8e,0x7,0x72,0x99,0x41,0x8c,0xe4,0xcb,0xc4,0x2,0x15,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0x7,0x0,0x6b,0x98,0xb0,0x68,0xc9,0x22,0x72,0x41,0x1b,0xd5,0xb,0x3b,0x7a,0x47,0x98,0x41,0xf6,0x33,0x86,0x45,0x6,0x4,0x0,0xd,0xf4,0xfb,0x26,0x6,0x1,0x8,0x7,0x0,0xc5,0x1c,0x77,0xf,0xe8,0x46,0x77,0x41,0xc2,0xde,0xec,0x5a,0x90,0x94,0x9e,0x41,0xc,0xbd,0x3f,0x45,0x2,0x1f,0x0,0x0,0xf4,0xfb,0x23,0x6,0x1,0x8,0x7,0x0,0xd9,0x27,0xc5,0xdc,0x51,0x5f,0x71,0x41,0x41,0x5b,0x7d,0xdf,0xc2,0x28,0x97,0x41,0xa7,0x6b,0x27,0x45,0x6,0xff,0x0,0x1,0x30,0x11,0x27,0x5,0x1,0x8,0xf,0x0,0xdc,0x19,0xbe,0xd0,0x6e,0x3d,0x76,0x41,0x2f,0x34,0xb3,0x4,0xcc,0x37,0x9d,0x41,0x44,0xca,0xfa,0xc4,0x2,0xf,0x0,0x0,0xf4,0xfb,0x25,0x5,0x1,0x8,0x7,0x0,0x86,0xdb,0xd6,0x4e,0xc2,0xe,0x72,0x41,0x6f,0x79,0x97,0x2b,0x31,0xb9,0x97,0x41,0xd,0xd4,0x35,0x45,0x0,0x1d,0x0,0x0,0xf4,0xfb,0x29,0x5,0x1,0x8,0x7,0x0,0x26,0xe9,0x53,0x9b,0xb3,0x37,0x72,0x41,0x6f,0xf4,0x7a,0xec,0xf9,0xee,0x97,0x41,0xa4,0x13,0x2a,0xc5,0x0,0x6,0x0,0x0,0xf4,0xfb,0x2a,0x5,0x1,0x8,0xf,0x0,0xe0,0x65,0xf9,0x72,0x8,0xdf,0x7f,0x41,0x94,0xd4,0xd0,0x99,0x7f,0xef,0xa4,0x41,0x78,0x1f,0x9b,0x43,0x1,0x8a,0x0,0x0,0xf4,0xfb,0x2b,0x5,0x1,0x8,0xf,0x0,0xd1,0x74,0x2f,0x92,0xc2,0xcd,0x72,0x41,0xe5,0xe7,0xd5,0x89,0x1e,0xb4,0x98,0x41,0xbc,0xdf,0x5e,0xc4,0x2,0xd,0x0,0x0,0xf4,0xfb,0x28,0x5,0x1,0x8,0x7,0x0,0xae,0x6f,0x55,0xc2,0xa4,0x3f,0x74,0x41,0xab,0x6,0x32,0x98,0xd,0x9a,0x9a,0x41,0x7a,0x3b,0x25,0xc5,0x0,0x18,0x0,0x0,0xf4,0xfb,0x24,0x6,0x1,0x8,0xf,0x0,0x3e,0xa4,0x61,0x85,0x9f,0x9f,0x70,0x41,0x3e,0xa4,0xf,0x7e,0x2f,0x33,0x96,0x41,0xca,0xe8,0x5a,0xc5,0x6,0x10,0x0,0x6,0xf4,0xfb,0x24,0x6,0x1,0x8,0xf,0x0,0xef,0x37,0x42,0x21,0xca,0x6f,0x70,0x41,0x9,0x31,0x4f,0x1b,0x5,0x98,0x95,0x41,0xea,0x10,0x91,0xc4,0x0,0x2,0x0,0x0,0xf4,0xfb,0x2c,0x5,0x1,0x8,0x7,0x0,0x61,0x6b,0xa5,0x4d,0x16,0x13,0x72,0x41,0x68,0x6d,0x8a,0xc9,0xc0,0x1c,0x98,0x41,0xda,0x67,0xae,0xc4,0x6,0x2,0x0,0x3,0xf4,0xfb,0x2a,0x5,0x1,0x8,0xf,0x0,0xe1,0x73,0x3e,0x87,0xeb,0x23,0x72,0x41,0x4e,0x95,0x47,0xe2,0x46,0x40,0x98,0x41,0x61,0x96,0x38,0x45,0x6,0x14,0x0,0x9,0xf4,0xfb,0x1e,0x8,0x3,0x8,0xf,0x0,0xcb,0xc8,0x82,0xb,0x70,0x56,0x76,0x41,0x7b,0x7b,0x3e,0xe1,0xa5,0x58,0x9d,0x41,0xbd,0x2f,0x35,0xc5,0x2,0x1b,0x0,0x0,0xf4,0xfb,0x24,0x5,0x1,0x8,0x7,0x0,0x7a,0x48,0xde,0xfa,0x97,0x19,0x74,0x41,0xd1,0x34,0xaf,0x8a,0x11,0x68,0x9a,0x41,0xf2,0x33,0xb7,0x44,0x0,0x1f,0x0,0x0,0xf4,0xfb,0x29,0x5,0x1,0x8,0xf,0x0,0xc3,0x4c,0xbf,0x3c,0x91,0x2e,0x70,0x41,0xf1,0x6c,0xab,0xa5,0x52,0x98,0x95,0x41,0x8c,0x45,0xe5,0xc4,0x6,0x12,0x0,0x4,0xf4,0xfb,0x2c,0x5,0x1,0x8,0xf,0x0,0x5e,0xee,0x27,0x65,0xc5,0xe7,0x6e,0x41,0xe8,0xc4,0xd,0x98,0x13,0x4d,0x94,0x41,0x33,0x7d,0x5a,0xc4,0x0,0xc,0x0,0x0,0xf4,0xfb,0x30,0x5,0x1,0x8,0xf,0x0,0x9c,0x9a };
            msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        }
        msg.ptype = _PTYPE_UBLOX;
        return true;
    }

    return false;
}

#include "message_stats.h"

bool GenerateRTCM3(test_message_t &msg, int i, float f)
{
    (void)f;
    
    if (periodCheck(s_msgTimeMs.rtcm1005, s_gpsPeriodMs))
    {   // RTCM3 (0x13 0x3e) 1005
        uint8_t buf[] = { 0xd3,0x0,0x13,0x3e,0xdc,0x2f,0x3,0x7b,0xcd,0x79,0xd5,0x47,0x35,0x77,0x5f,0x93,0x4d,0x49,0x8f,0xf1,0xb3,0x1d,0xff,0x10,0x3d };
        msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        msg.ptype = _PTYPE_RTCM3;
        return true;
    }

    if (periodCheck(s_msgTimeMs.rtcm1007, s_gpsPeriodMs))
    {   // RTCM3 (0x19 0x3e) 1007
        uint8_t buf[] = { 0xd3,0x0,0x19,0x3e,0xfc,0x2f,0x14,0x41,0x44,0x56,0x4e,0x55,0x4c,0x4c,0x41,0x4e,0x54,0x45,0x4e,0x4e,0x41,0x20,0x20,0x4e,0x4f,0x4e,0x45,0x0,0xc4,0xe,0xe1 };
        msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        msg.ptype = _PTYPE_RTCM3;
        return true;
    }

    if (periodCheck(s_msgTimeMs.rtcm1033, s_gpsPeriodMs))
    {   // RTCM3 (0x48 0x40) 1033
        uint8_t buf[] = { 0xd3,0x0,0x48,0x40,0x9c,0x2f,0x14,0x41,0x44,0x56,0x4e,0x55,0x4c,0x4c,0x41,0x4e,0x54,0x45,0x4e,0x4e,0x41,0x20,0x20,0x4e,0x4f,0x4e,0x45,0x0,0x0,0xd,0x54,0x52,0x49,0x4d,0x42,0x4c,0x45,0x20,0x4e,0x45,0x54,0x52,0x39,0x14,0x4e,0x61,0x76,0x20,0x34,0x2e,0x36,0x32,0x20,0x2f,0x20,0x42,0x6f,0x6f,0x74,0x20,0x34,0x2e,0x36,0x32,0xa,0x35,0x33,0x32,0x39,0x4b,0x34,0x34,0x33,0x35,0x32,0xfc,0xca,0x3f };
        msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        msg.ptype = _PTYPE_RTCM3;
        return true;
    }

    if (periodCheck(s_msgTimeMs.rtcm1085, s_gpsPeriodMs))
    {   // RTCM3 (0xcc 0x43) 1085
        uint8_t buf[] = { 0xd3,0x0,0xcc,0x43,0xdc,0x2f,0x82,0x2f,0xc0,0xe2,0x0,0x20,0x4,0x3,0x41,0x0,0x0,0x0,0x0,0x0,0x30,0xc0,0x0,0x0,0x7f,0xff,0xfa,0x4a,0x1a,0x22,0x2a,0x54,0x3,0xdd,0x28,0xec,0x62,0x15,0xe,0xbb,0x90,0x0,0x40,0x1,0x0,0x4,0x0,0x10,0x0,0x75,0x28,0xea,0x45,0xd6,0x33,0xac,0x20,0x91,0x41,0x1d,0xa2,0x6b,0x84,0xcc,0xed,0x28,0xda,0xb,0xb5,0xc3,0x6b,0x87,0x2f,0x3e,0x57,0x1c,0xe3,0x79,0xd9,0x10,0xec,0x21,0xaa,0x47,0xa0,0x90,0xdf,0xff,0xfb,0x5f,0xff,0xee,0x7f,0xff,0xaf,0xff,0xfe,0xaf,0xff,0xfa,0xdf,0xff,0xea,0x0,0x1,0xa2,0x0,0x6,0x68,0x0,0xf,0x40,0x0,0x41,0x0,0x0,0x5c,0x0,0x1,0x68,0x0,0x14,0x9f,0xff,0xaf,0x0,0x1,0x2e,0x0,0x4,0xc0,0x0,0x13,0xe0,0x0,0x52,0x0,0x1,0x8a,0x0,0x6,0x10,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x5f,0x75,0x96,0x67,0x96,0x38,0x65,0x8d,0xd7,0x5f,0x75,0x95,0xd9,0x64,0xd2,0xc0,0x0,0x80,0x1,0x0,0x2,0x0,0x4,0x0,0x8,0x0,0x10,0x0,0x20,0x0,0x40,0x0,0x80,0x1,0x0,0x2,0x0,0x4,0x0,0x8,0x0,0x10,0x0,0x20,0x0,0x40,0x0,0x80,0x1,0x0,0x2,0x0,0x0,0x9d,0xc4,0x6a };
        msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        msg.ptype = _PTYPE_RTCM3;
        return true;
    }

    if (periodCheck(s_msgTimeMs.rtcm1095, s_gpsPeriodMs))
    {   // RTCM3 (0x3a 0x44) 1095
        uint8_t buf[] = { 0xd3,0x1,0x3a,0x44,0x7c,0x2f,0x52,0x3,0x6c,0x22,0x0,0x20,0x34,0x98,0x0,0xc0,0x0,0x0,0x0,0x0,0x4,0x0,0x88,0x80,0x7f,0xff,0xff,0xff,0xac,0x2a,0xad,0x29,0xa9,0xa7,0xa6,0xa7,0x80,0x0,0x0,0x0,0x6e,0x64,0x72,0xc7,0x4a,0xea,0x56,0xa3,0x32,0xfe,0xc0,0x1,0x0,0x4,0x0,0x10,0x0,0x40,0x1,0x0,0x4,0x0,0x10,0x0,0x3,0xf7,0x8,0x96,0x12,0x90,0x23,0xd7,0x25,0xbe,0x66,0xbc,0xd0,0x79,0xb9,0xc,0xbb,0x1b,0x5e,0x37,0x74,0x6d,0xe2,0x1d,0x94,0x45,0xc8,0xa2,0x51,0x5d,0xe3,0x3a,0xc6,0x81,0x8d,0xbb,0x19,0x8e,0x7b,0x5c,0xf3,0xd9,0xf1,0xf3,0xe0,0x6c,0x46,0xde,0x85,0xbd,0x4f,0x7c,0x1f,0x4,0x9e,0x16,0xfc,0x44,0xb8,0x81,0xff,0xff,0xf5,0xff,0xff,0x8f,0xff,0xeb,0xc0,0x0,0x15,0x7f,0xfe,0xe5,0xff,0xfa,0x28,0x0,0x11,0xdf,0xff,0xde,0x7f,0xfe,0xd8,0x0,0x1,0xcf,0xff,0xeb,0xdf,0xff,0xf1,0xff,0xff,0x8,0x0,0x6,0x10,0x0,0x6,0x3f,0xff,0x9c,0x80,0x0,0x47,0xff,0xfe,0xf0,0x0,0x14,0xff,0xff,0xa6,0x0,0x1,0x29,0xff,0xfc,0x58,0x0,0x9,0x0,0x0,0x56,0xff,0xff,0xe5,0xff,0xfb,0x50,0x0,0x12,0x3f,0xff,0xcc,0xff,0xff,0x2b,0xff,0xfb,0x17,0xff,0xfa,0x3f,0xff,0xf1,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x59,0x5e,0x15,0x61,0x6e,0xb7,0x55,0x66,0x35,0xd5,0x56,0x36,0xd9,0x5e,0x36,0x59,0x56,0x15,0xe3,0x7e,0xf8,0x65,0x87,0x19,0x40,0x0,0x80,0x1,0x0,0x2,0x0,0x4,0x0,0x8,0x0,0x10,0x0,0x20,0x0,0x40,0x0,0x80,0x1,0x0,0x2,0x0,0x4,0x0,0x8,0x0,0x10,0x0,0x20,0x0,0x40,0x0,0x80,0x1,0x0,0x2,0x0,0x4,0x0,0x8,0x0,0x10,0x0,0x20,0x0,0x40,0x0,0x80,0x1,0x0,0x2,0x0,0x4,0x0,0x8,0x0,0x10,0x0,0x20,0x0,0x0,0x1d,0x23,0x6c };
        msg.pktSize = sizeof(buf);  memcpy(msg.comm.rxBuf.start, buf, msg.pktSize);
        msg.ptype = _PTYPE_RTCM3;
        return true;
    }

    return false;
}

bool ptypeEnabled(protocol_type_t ptype)
{
    // Uncomment messages to enable them
    switch (ptype)
    {
    default:
        return false;

    case _PTYPE_INERTIAL_SENSE_DATA:
    case _PTYPE_INERTIAL_SENSE_CMD:
    case _PTYPE_NMEA:
    case _PTYPE_UBLOX:
    case _PTYPE_RTCM3:
        return true;
    }
}

// This function returns true so long as there is a valid message for the given timestamp
bool GenerateMessage(test_message_t &msg, protocol_type_t ptype)
{
    // Random number engine (using Mersenne Twister algorithm)
    static std::mt19937 rng(std::random_device{}());

    // Uniform distribution in range [MIN_VALUE, MAX_VALUE]
    static std::uniform_int_distribution<int> dist(MIN_VALUE, MAX_VALUE);

    // Generate random number
    int i = dist(rng);
    float f = (float)i * 0.001f;

    // GenerateISB() must run to generate ISB data used in GenerateNMEA()
    if (GenerateISB(msg, i, f))
    {
        if (ptype == _PTYPE_NONE || ptype == _PTYPE_INERTIAL_SENSE_DATA)
        {
            msg.pktSize = is_comm_data_to_buf(msg.comm.rxBuf.start, msg.comm.rxBuf.size, &msg.comm, msg.dataHdr.id, msg.dataHdr.size, 0, (void*)&msg.data);
        }
        else
        {
            msg.pktSize = 0;
        }
        return true;
    }

    if ((ptype == _PTYPE_NONE || ptype == _PTYPE_NMEA) && GenerateNMEA(msg, i, f))
    {
        msg.dataHdr.size = msg.comm.rxPkt.dataHdr.size = msg.pktSize;
        msg.comm.rxBuf.tail = msg.comm.rxBuf.start + msg.pktSize;
        return true;
    }

    if ((ptype == _PTYPE_NONE || ptype == _PTYPE_UBLOX) && GenerateUblox(msg, i, f))
    {
        msg.dataHdr.size = msg.comm.rxPkt.dataHdr.size = msg.pktSize;
        msg.comm.rxBuf.tail = msg.comm.rxBuf.start + msg.pktSize;
        return true;
    }

    if ((ptype == _PTYPE_NONE  || ptype == _PTYPE_RTCM3) && GenerateRTCM3(msg, i, f))
    {
        msg.dataHdr.size = msg.comm.rxPkt.dataHdr.size = msg.pktSize;
        msg.comm.rxBuf.tail = msg.comm.rxBuf.start + msg.pktSize;
        return true;
    }

    return false;
}

void GenerateDataLogFiles(int numDevices, string directory, cISLogger::eLogType logType, float logSizeMB, eTestGenDataOptions options)
{
    // Remove old files
	ISFileManager::DeleteDirectory(directory);

    cISLogger logger;
    logger.InitSave(logType, directory, numDevices, s_maxDiskSpacePercent, s_maxFileSize, s_useTimestampSubFolder);
    for (int d=0; d<numDevices; d++)
    {   // Assign serial number
        dev_info_t info = {};
        info.serialNumber = 100000 + d;
        logger.SetDeviceInfo(&info, d);
    }
    logger.EnableLogging(true);
    logger.ShowParseErrors(options != GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS);

    test_message_t msg = {};
    uint8_t comBuf[PKT_BUF_SIZE];
    is_comm_init(&msg.comm, comBuf, PKT_BUF_SIZE);

    CurrentGpsTimeMs(s_gpsTowOffsetMs, s_gpsWeek);

    for (s_timeMs=0; logger.LogSizeMB() < logSizeMB; s_timeMs += s_timePeriodMs)
    {
        for (int d=0; d<numDevices; d++)
        {
            while(GenerateMessage(msg))
            {
                // Write data to file
                if (logType == cISLogger::eLogType::LOGTYPE_RAW)
                {
                    logger.LogData(d, msg.pktSize, msg.comm.rxBuf.start);
                }
                else // cISLogger::eLogType::LOGTYPE_DAT
                {
                    logger.LogData(d, &msg.dataHdr, (const uint8_t*)&msg.data);
                }

                // Insert garbage data
                static int pktCount = 0;
                if (options == GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS && pktCount++ > 10)
                {
                    pktCount = 0;
                    uint8_t garbage[100];

                    for (int i=0; i<sizeof(garbage); i++)
                    {   // Generate garbage
                        garbage[i] = rand();
                    }
                    int garbageSize = garbage[0]%100;
                    
                    logger.LogData(d, garbageSize, garbage);
                }
            }
        }
    }

    logger.CloseAllFiles();
}

bool AddDataToStream(uint8_t *buffer, int bufferSize, int &streamSize, uint8_t *data, int dataSize)
{
    if (streamSize + dataSize < bufferSize)
    {   // Data fits in buffer.  Write data to stream.
        memcpy(buffer + streamSize, data, dataSize);
        streamSize += dataSize;
        return true;
    }

    return false;
}

int GenerateDataStream(uint8_t *buffer, int bufferSize, eTestGenDataOptions options)
{
    test_message_t msg = {};
    uint8_t comBuf[PKT_BUF_SIZE];
    is_comm_init(&msg.comm, comBuf, PKT_BUF_SIZE);
    int streamSize = 0;
    static int pktCount = 0;

    CurrentGpsTimeMs(s_gpsTowOffsetMs, s_gpsWeek);

    for (s_timeMs=0;; s_timeMs += s_timePeriodMs)
    {
        while(GenerateMessage(msg))
        {
            if (msg.pktSize == 0)
            {   // Ignore empty data
                continue;
            }

            pktCount++;

            // Truncate message end
            if ((options & GEN_LOG_OPTIONS_MISSING_MESSAGE_END) && (pktCount%28 == 0))
            {
                msg.pktSize /= 2;
            }

            // Add data to steam
            if (!AddDataToStream(buffer, bufferSize, streamSize, msg.comm.rxBuf.start, msg.pktSize))
            {   // Buffer full
                return streamSize;
            }

            // Duplicate data
            if ((options & GEN_LOG_OPTIONS_TIMESTAMP_DUPLICATE) && (pktCount%23 == 0))
            {   
                DEBUG_PRINT("ADDING DUPLICATE: \n");
                DEBUG_PRINT("      %.*s", msg.pktSize, msg.comm.rxBuf.start);
                if (!AddDataToStream(buffer, bufferSize, streamSize, msg.comm.rxBuf.start, msg.pktSize))
                {   // Buffer full
                    return streamSize;
                }
            }

            // Cause timestamp to momentarily reverse 
            if ((options & GEN_LOG_OPTIONS_TIMESTAMP_REVERSE) && (pktCount%18 == 0))
            {
                DEBUG_PRINT("ADDING REVERSAL: \n");
                s_timeMs -= s_gpsPeriodMs;
                s_msgTimeMs.gpsPos = 0;
                s_msgTimeMs.ins1 = 0;
                s_msgTimeMs.nmeaGpsPos = 0;
                s_msgTimeMs.nmeaZda = 0;
                s_msgTimeMs.nmeaGga = 0;
                s_msgTimeMs.nmeaIns1 = 0;
            }

            // Insert garbage data
            if (options & GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS && (pktCount%15 == 0))
            {
                DEBUG_PRINT("ADDING GARBAGE: \n");
                uint8_t garbage[100];

                for (int i=0; i<sizeof(garbage); i++)
                {   // Generate garbage
                    garbage[i] = rand();
                }
                int garbageSize = garbage[0]%100;

                if (!AddDataToStream(buffer, bufferSize, streamSize, garbage, garbageSize))
                {   // Buffer full
                    return streamSize;
                }
            }
        }
    }

    return streamSize;
}