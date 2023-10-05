#ifndef RADAR_H_
#define RADAR_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Float32.h>
#include <radar_msgs/RadarDetectionArray.h>
#include <radar_msgs/RadarDetection.h>

#include "radar/mr76.h"
#include "radar/common.h"
#include "radar/params.h"

class Radar
{

public:
    Radar(ros::NodeHandle *node_handle, int16_t radar_id = 0); // constructor
    radar_msgs::RadarDetectionArray MakeMessage(const RadarData &data);
    void CanCallback(const can_msgs::Frame &frame);

private:
    ros::NodeHandle *nh;
    ros::Subscriber sub_can_;              // subscribers to incomming can data
    ros::Publisher pub_radar_out_;         // radar ouput

    int16_t hardware_id;                                 // radar id if in multiple radars present
    Mr76Driver mr76_radar_;                              // mr76 radar driver
    RadarData curr_radar_out_;                           // latest extracted scene
    RadarData radar_out_;                                // published radar out
    radar_msgs::RadarDetectionArray detections_out_msg_; // message
    Parameters params_;                             //input parameters                               

    void InitSub();           // initialises subscribers
    void InitPub();           // initialises publishers
    void GetParams();         // retrieves parameters from parameter server
    void InitDefaultValues(); // initialise default values
    template <typename T>
    bool IsInBounds(const T &value, const T &low, const T &high);
    template <typename T>
    bool IsInRegion(const TargetData &target,
                    const T &min_lat_dist, const T &max_lat_dist,
                    const T &min_long_dist, const T &max_long_dist);
    template <typename T>
    void RemoveOutsideObjects(RadarData &radar_data,
                      const T &min_lat_dist, const T &max_lat_dist,
                      const T &min_long_dist, const T &max_long_dist);

    RadarData PostProcess(RadarData &raw_data); // post process the data
};

#endif // RADAR_H