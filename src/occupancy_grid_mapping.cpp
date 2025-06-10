#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <fstream>
#include <random>
#include "occupancy_grid_mapping.hpp"

constexpr float LOG_ODDS_OCCUPANCY_THRESHOLD { 1.0f };
constexpr float LOG_ODDS_FREE { -1.0f };
constexpr float LOG_ODDS_OCCUPIED { 3.0f };
constexpr float MIN_LOG_ODDS = -2.0f;
constexpr float MAX_LOG_ODDS = 6.0f;

OccupancyGridMap::OccupancyGridMap(int width, int height, float resolution)
    : width_(width), height_(height), resolution_(resolution) {
    log_odds_grid_ = std::vector<std::vector<float>>(height,
                     std::vector<float>(width, -0.5f));
}

bool OccupancyGridMap::worldToMap(float wx, float wy, int& mx, int& my) const {
    mx = static_cast<int>(wx / resolution_);
    my = static_cast<int>(wy / resolution_);
    return mx >= 0 && my >= 0 && mx < width_ && my < height_;
}

bool OccupancyGridMap::mapToWorld(int mx, int my, float& wx, float& wy) const{
    if (mx < 0 || my < 0 || mx >= width_ || my >= height_) return false;

    wx = mx * resolution_;
    wy = my * resolution_;
    return true;
}

float OccupancyGridMap::getCell(int x, int y) const {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return 0.0f;
    return log_odds_grid_[y][x];
}

void OccupancyGridMap::updateCell(int x, int y, float log_odds){
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
    // Clamp the log odds to avoid growing without bounds
    log_odds_grid_[y][x] = std::clamp(log_odds_grid_[y][x] + log_odds, MIN_LOG_ODDS, MAX_LOG_ODDS);
}

float OccupancyGridMap::rayCasting(const Pose& pose, float ray_angle, float max_range) const{
    int mx, my;
    worldToMap(pose.x, pose.y, mx, my);
    float dx { static_cast<float>(cos(ray_angle + pose.theta))};
    float dy { static_cast<float>(sin(ray_angle + pose.theta))};

    float step { 0.02f }; // meters per step
    float distance { 0.0f }; // distance traversed by the ray
    float x { pose.x };
    float y { pose.y };
    while (distance < max_range)
    {
        distance += step;
        x += dx * step;
        y += dy * step;
        if (!worldToMap(x, y, mx, my))
            break;
        else
        {
            if (getCell(mx, my) > LOG_ODDS_OCCUPANCY_THRESHOLD) {
                return distance;
            }
        }
    }
    return max_range; // no hit or break (out of bounds)
}

Eigen::Vector2f OccupancyGridMap::computeRayEndpoint(const Pose& pose, float ray_angle, float distance) {
    float ray_angle_world = pose.theta + ray_angle;  // global angle
    Eigen::Vector2f origin{pose.x, pose.y};
    Eigen::Vector2f direction{cos(ray_angle_world), sin(ray_angle_world)};
    return origin + distance * direction;
}

void OccupancyGridMap::updateWithScan(const Pose& pose, const std::vector<float>& measurements, const std::vector<float>& angles){
    int x0, y0;
    int x1, y1;
    const float max_range {5.0f};
    for (size_t i {0}; i < measurements.size(); ++i)
    {
        if (measurements[i] < 0.0f || measurements[i] > max_range)
            continue; // ignore invalid distances

        float corrected_distance = std::max(0.0f, measurements[i] - resolution_);
        Eigen::Vector2f endpoint = computeRayEndpoint(pose, angles[i], corrected_distance);
        if (!worldToMap(pose.x, pose.y, x0, y0) || !worldToMap(endpoint.x(), endpoint.y(), x1, y1))
            continue; // Skip invalid rays

        int dx { abs(x1 - x0) };
        int dy { abs(y1 - y0) };

        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;

        int err = dx - dy;
        int e2;

        while (!(x0 == x1 && y0 == y1))
        {
            updateCell(x0, y0, LOG_ODDS_FREE);
            e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }

        // Mark endpoint occupied only if measurement is less than max range minus margin
        if (measurements[i] < max_range - 0.1f) {
            updateCell(x1, y1, LOG_ODDS_OCCUPIED);
        }
    }
}

// Generates a vector of ray angles (in degrees) evenly spread over the field of view.
// Example: 19 rays over 180 degrees from -90 to +90 degrees.
std::vector<float> OccupancyGridMap::generateScanAngles(int num_rays = 19, float fov_deg = 180.0f) {
    std::vector<float> angles;
    float angle_increment = fov_deg / (num_rays - 1);
    for (int i = 0; i < num_rays; ++i) {
        angles.push_back(static_cast<float>(-fov_deg / 2 + i * angle_increment) * M_PI / 180);
    }
    return angles;
}

// Creates a simple robot trajectory as a vector of poses.
// Example: Robot moves in a straight line along the x-axis with zero orientation.
std::vector<Pose> OccupancyGridMap::generateTrajectory(int num_poses = 50) {
    std::vector<Pose> poses;
    float radius = 3.0f;
    for (int i = 0; i < num_poses; ++i) {
        float angle = 2 * M_PI * i / num_poses;
        float x = radius * cos(angle) + 5.0f; // Centered at (5,5)
        float y = radius * sin(angle) + 5.0f;
        float theta = angle + M_PI; // face inward
        poses.push_back(Pose{x, y, theta});
    }
    return poses;
}

// Simulates range sensor scans at each pose by ray casting on the map.
// For each pose and each ray angle, performs ray casting and records the distance.
std::vector<std::vector<float>> OccupancyGridMap::simulateScans(const std::vector<Pose>& poses,
                                              const std::vector<float>& angles,
                                              float max_range) const {
    std::vector<std::vector<float>> scans;
    for (const auto& pose : poses) {
        std::vector<float> scan;
        for (float angle : angles) {
            // Convert angle to radians inside rayCasting
            float dist = rayCasting(pose, angle, max_range);
            scan.push_back(dist);
        }
        scans.push_back(scan);
    }
    return scans;
}

void OccupancyGridMap::saveToCSV(const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing.\n";
        return;
    }

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float log_odds = log_odds_grid_[y][x];
            float prob = 1.0f / (1.0f + std::exp(-log_odds));
            ofs << prob;
            if (x < width_ - 1)
                ofs << ",";
        }
        ofs << "\n";
    }
    ofs.close();
}

int main() {
    // 1. Simulate on a map with a known wall
    OccupancyGridMap sim_map(100, 100, 0.1f);
    // Create a simulated world with a box
    for (int x = 20; x < 80; ++x) {
        sim_map.updateCell(x, 20, LOG_ODDS_OCCUPIED); // top
        sim_map.updateCell(x, 80, LOG_ODDS_OCCUPIED); // bottom
    }
    for (int y = 20; y < 80; ++y) {
        sim_map.updateCell(20, y, LOG_ODDS_OCCUPIED); // left
        sim_map.updateCell(80, y, LOG_ODDS_OCCUPIED); // right
    }

    // 2. Empty map to build up via scans
    OccupancyGridMap mapping_map(100, 100, 0.1f);

    // 3. Generate angles and trajectory
    auto angles = sim_map.generateScanAngles(180); // Already in radians
    auto poses = sim_map.generateTrajectory(200);

    // 4. Simulate scans on sim_map with noise
    std::default_random_engine gen;
    std::normal_distribution<float> noise(0.0f, 0.01f); // 1cm noise

    auto scans = sim_map.simulateScans(poses, angles, 5.0f);
    for (auto& scan : scans)
        for (auto& d : scan)
            d += noise(gen);

    // 5. Feed scans into mapping_map
    for (size_t i = 0; i < poses.size(); ++i)
        mapping_map.updateWithScan(poses[i], scans[i], angles);

    // Add here to print sample cells' log-odds values
    std::cout << "Sample log-odds values after updates:\n";
    for (int y = 15; y <= 25; ++y) {
        for (int x = 15; x <= 25; ++x) {
            std::cout << mapping_map.getCell(x, y) << " ";
        }
        std::cout << "\n";
    }

    // 6. Save the reconstructed map
    mapping_map.saveToCSV("map.csv");

    std::cout << "Saved occupancy grid as map.csv\n";

    std::cout << "Mapping finished." << std::endl;
    return 0;
}