#include "radar/radar.h"

Radar::Radar(ros::NodeHandle *node_handle, int16_t radar_id)
{
    nh = node_handle;
    hardware_id = radar_id;

    InitSub();           // initialise subscribers
    InitPub();           // initialise publishers
    GetParams();         // get parameters from the yaml file
    InitDefaultValues(); // initailise default values
}

// initialise subscribers
void Radar::InitSub()
{

    sub_can_ = nh->subscribe("/received_messages", 300,
                             &Radar::CanCallback, this);
}

// initialise publishers
void Radar::InitPub()
{
    pub_radar_out_ = nh->advertise<radar_msgs::RadarDetectionArray>("detections", 10);
}

// extract required parameters from the parameter server
void Radar::GetParams()
{
    bool success = true;
    // and and and
    success = success && nh->getParam("use_pathbased_filter", params_.use_pathbased_filter);
    success = success && nh->getParam("path_width_meters", params_.path_width);
    success = success && nh->getParam("path_length_meters", params_.path_length);

    if (!success)
    {
        ROS_ERROR("RADAR: Failed to extract all values from param file");
    }
}

// initialise default values
void Radar::InitDefaultValues()
{
}

// checks if the point is in the bounds
template <typename T>
bool Radar::IsInBounds(const T &value, const T &low, const T &high)
{
    return (value >= low) && (value <= high);
}

// check if the target is in the specified region
template <typename T>
bool Radar::IsInRegion(const TargetData &target,
                       const T &min_lat_dist, const T &max_lat_dist,
                       const T &min_long_dist, const T &max_long_dist)
{
    return (IsInBounds(target.dist_lat, min_lat_dist, max_lat_dist) && IsInBounds(target.dist_long, min_long_dist, max_long_dist));
}

// removes the points outside the egos path
template <typename T>
void Radar::RemoveOutsideObjects(RadarData &radar_data,
                                 const T &min_lat_dist, const T &max_lat_dist,
                                 const T &min_long_dist, const T &max_long_dist)
{
    std::vector<TargetData> filtered_targets;

    for (auto target : radar_data.targets)
    {
        if (IsInRegion(target, min_lat_dist, max_lat_dist, min_long_dist, max_long_dist))
        {
            filtered_targets.push_back(target);
        }
    }
    radar_data.targets = filtered_targets;
}

// Post process the radar data
RadarData Radar::PostProcess(RadarData &raw_data)
{

    RadarData processed_data = raw_data;

    if (params_.use_pathbased_filter)
    {
        // path is asumed to be equally flexed at eighter region of the radar
        RemoveOutsideObjects(processed_data,
                             -(params_.path_width / 2),
                             +(params_.path_width / 2),
                             (float)0, params_.path_length);
    }
    processed_data.total_targets = processed_data.targets.size();
    return processed_data;
}

// copy the data to message
radar_msgs::RadarDetectionArray Radar::MakeMessage(const RadarData &data)
{
    radar_msgs::RadarDetection target_msg;
    radar_msgs::RadarDetectionArray out_msg;

    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "radar";

    for (int16_t i = 0; i < data.targets.size(); i++)
    {
        target_msg.detection_id = data.targets[i].id;
        target_msg.position.x = data.targets[i].dist_long;
        target_msg.position.y = data.targets[i].dist_lat;
        target_msg.velocity.x = data.targets[i].vel_rel_long;
        target_msg.velocity.y = data.targets[i].vel_rel_lat;
        target_msg.amplitude = data.targets[i].rcs;

        out_msg.detections.push_back(target_msg);
    }

    return out_msg;
}

// CAN message callback /received_messages'
void Radar::CanCallback(const can_msgs::Frame &frame)
{
    bool scene_extracted = mr76_radar_.ExtractDetections(frame, curr_radar_out_);

    if (scene_extracted) // successfull scene extraction
    {
        radar_out_ = PostProcess(curr_radar_out_);
        detections_out_msg_ = MakeMessage(radar_out_);
        pub_radar_out_.publish(detections_out_msg_);
    }
}