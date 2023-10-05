#ifndef RADAR_COMMON_H_
#define RADAR_COMMON_H_

const int kCanDataSize = 8; // can data is 8 bytes

// target properties
struct TargetData
{
    int16_t id;           // object id
    int16_t dynamic_prop; // dynamic properties
    float dist_long;      // longitudinal distance
    float dist_lat;       // latitudinal distance
    float vel_rel_long;   // relative velocity in longitudinal direction
    float vel_rel_lat;    // relative velocity in latitudinal direction
    float rcs;
};

// mr76 radar properties
struct RadarData
{
    int16_t radar_state;
    int32_t total_targets;
    std::vector<TargetData> targets;
};

inline void PrintData(const TargetData &data)
{
    std::cout << "ID: " << data.id << std::endl;
    std::cout << "Dist Long: " << data.dist_long << std::endl;
    std::cout << "Dist Lat: " << data.dist_lat << std::endl;
    std::cout << "Vel Long: " << data.vel_rel_long << std::endl;
    std::cout << "Vel Lat: " << data.vel_rel_lat << std::endl;
    std::cout << "Dynamic prop: " << data.dynamic_prop << std::endl;
    std::cout << "RCS: " << data.rcs << std::endl;
    std::cout << "===========================================================================" << std::endl;
}

#endif // RADAR_COMMON_H_