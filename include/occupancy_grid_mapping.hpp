#include <vector>

struct Pose
{
    float x, y, theta;
};

class OccupancyGridMap {
public:
    OccupancyGridMap(int width, int height, float resolution);

    void updateCell(int x, int y, float log_odds);
    float getCell(int x, int y) const;

    // Convert world coordinates to grid indices and viceversa
    bool worldToMap(float wx, float wy, int& mx, int& my) const;
    bool mapToWorld(int mx, int my, float& wx, float& wy) const;

    float rayCasting(const Pose& pose, float ray_angle, float max_range) const;
    void updateWithScan(const Pose& pose, const std::vector<float>& ranges, const std::vector<float>& angles);

    std::vector<float> generateScanAngles(int num_rays, float fov_deg);
    std::vector<Pose> generateTrajectory(int num_poses);
    std::vector<std::vector<float>> simulateScans(const std::vector<Pose>& poses,
                                              const std::vector<float>& angles,
                                              float max_range) const;
    
    void saveToCSV(const std::string& filename);
private:
    int width_, height_;
    float resolution_; // meters per cell
    std::vector<std::vector<float>> log_odds_grid_;

    Eigen::Vector2f computeRayEndpoint(const Pose& pose, float ray_angle, float distance);
};
