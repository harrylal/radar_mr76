#include "radar/mr76.h"

Mr76Driver::Mr76Driver()
{
}

// returns true if scene extraction sucess
bool Mr76Driver::ExtractDetections(const can_msgs::Frame &frame, RadarData &radar_out)
{

    switch (frame.id)
    {
    case MessageID::kIdObjStatus:

        if (true == buffer_manager_.IsReached()) // successfull scene extraction
        {
            radar_out = data_parsed;     // copy parsed
            data_parsed.targets.clear(); // empty vector
            buffer_manager_.Reset();

            buffer_manager_.SetPeak(frame.data[0]);    // prepare for next extraction
            data_parsed.total_targets = frame.data[0]; // prepare for next extraction
            return true;
        }
        else
        {
            std::cout << "[BUFFER WARN]: Skipping Scene" << std::endl;

            data_parsed.total_targets = 0;
            data_parsed.targets.clear();
            buffer_manager_.Reset();
            buffer_manager_.SetPeak(frame.data[0]);
        }
        break;

    case MessageID::kIdObjGeneral:

        data_parsed.targets.push_back(ParseTargetInfo(frame));
        buffer_manager_.UpdateCounter();
        break;

    default:
        break;
    }

    return false;
}

int16_t Mr76Driver::ParseTotalObjects(const can_msgs::Frame &frame)
{
    return (int16_t)frame.data[0];
}

// parses the target information
TargetData Mr76Driver::ParseTargetInfo(const can_msgs::Frame &frame)
{
    int16_t data_stream[8]; // stream of extracted bits defining data
    TargetData parsed_output_;

    data_stream[0] = frame.data[0];                                                            // object id (Checked)
    data_stream[1] = int16_t(((frame.data[1] << 8) | (frame.data[2] & 0xF8)) >> 3);            // distance long (Check)
    data_stream[2] = int16_t((frame.data[2] & 0x07) << 8 | frame.data[3]);                     // distance lat (Checked)
    data_stream[3] = int16_t(((frame.data[4] << 8) | (frame.data[5] & 0xC0)) >> 6);            // Vrelative long (Checked)
    data_stream[4] = int16_t(((((frame.data[5] & 0x3F) << 8) | (frame.data[6] & 0xE0))) >> 5); // Vrelative lat (Checked)
    data_stream[5] = int16_t((frame.data[6] & 0x18) >> 3);                                     // Object class (Checked)
    data_stream[6] = int16_t(frame.data[6] & 0x07);                                            // Object dynamic prop (Checked
    data_stream[7] = frame.data[7];                                                            // Object RCS (Checked)

    parsed_output_.id = data_stream[0] * 1;
    parsed_output_.dist_long = data_stream[1] * 0.2 - 500;
    parsed_output_.dist_lat = data_stream[2] * 0.2 - 204.6;
    parsed_output_.vel_rel_long = data_stream[3] * 0.25 - 128.0;
    parsed_output_.vel_rel_lat = data_stream[4] * 0.25 - 64.0;
    parsed_output_.dynamic_prop = data_stream[6] * 1;
    parsed_output_.rcs = data_stream[7] * 0.5 - 64.0;

    return parsed_output_;
}

BufferHandle::BufferHandle()
{
    peak_ = 0;
    counter_ = 0;
}

void BufferHandle::Reset()
{
    peak_ = 0;
    counter_ = 0;
}

void BufferHandle::SetPeak(int32_t new_peak)
{
    peak_ = new_peak;
}

void BufferHandle::UpdateCounter()
{
    ++counter_;
}

uint32_t BufferHandle::GetCounter()
{
    return counter_;
}

bool BufferHandle::IsReached()
{
    return (counter_ == peak_ ? true : false);
}

bool BufferHandle::IsOverFlow()
{
    return (counter_ > peak_ ? true : false);
}

bool BufferHandle::IsUnderFlow()
{
    return (counter_ < peak_ ? true : false);
}
