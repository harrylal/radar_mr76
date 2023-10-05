#ifndef MR76_H_
#define MR76_H_

#include <can_msgs/Frame.h>

#include "radar/common.h"


class BufferHandle
{
    uint32_t peak_;
    uint32_t counter_;

public:
    BufferHandle();
    void SetPeak(int32_t new_peak);
    void Reset();
    void UpdateCounter();
    uint32_t GetCounter();
    bool IsReached();
    bool IsOverFlow();
    bool IsUnderFlow();
};


class Mr76Driver
{

public:
    // enum can message ids
    enum MessageID : unsigned int
    {
        kIdRadarState = 0x21,
        kIdSoftwareVersion = 0x700,
        kIdObjStatus = 0x60A,
        kIdObjGeneral = 0x60B,
    };


    BufferHandle buffer_manager_;
    RadarData data_parsed;

    Mr76Driver();
    TargetData ParseTargetInfo(const can_msgs::Frame &frame);
    int16_t ParseTotalObjects(const can_msgs::Frame &frame);
    bool ExtractDetections(const can_msgs::Frame &frame, RadarData &radar_out);
};

#endif // MR76_H