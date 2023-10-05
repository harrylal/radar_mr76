#ifndef RADAR_PARAMS_H_
#define RADAR_PARAMS_H_

// encapsulation for all external parameters retrieved from yaml file
struct Parameters
{
    bool use_pathbased_filter = false; // filter targets outside the path
    float path_width = 30;            // path length within the radar
    float path_length = 200;          // range to consider

};

#endif // RADAR_PARAMS_H_
