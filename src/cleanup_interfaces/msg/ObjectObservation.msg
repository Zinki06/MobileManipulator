std_msgs/Header header
string observation_id
string object_uuid
string class_name
float32 confidence
int32[4] bbox
geometry_msgs/Point centroid
float32 position_uncertainty
string station_name
uint8 heading_index
string image_reference
int64 detector_track_id
uint16 confirmation_count
bool grasp_valid
string grasp_reason
float32[2] grasp_pixel

# candidate_body is required for execution; candidate_approach is navigation-only.
string grasp_strategy
