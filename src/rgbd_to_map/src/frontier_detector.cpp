#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_msgs/msg/frontier_info_extended.hpp"
#include "frontier_msgs/msg/frontier_array_extended.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "semantic_grid_msgs/msg/semantic_grid.hpp"

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <future>
#include <thread>
#include <sstream>
#include <limits>
#include <chrono>
#include <cstdint>
#include <functional>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class FrontierDetector : public rclcpp::Node
{
public:
    FrontierDetector() : Node("frontier_detector"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        declare_parameter<double>("update_rate", 1.0);
        declare_parameter<std::string>("frontier_backend", "wavefront_grid");
        declare_parameter<double>("dbscan_eps", 0.25);
        declare_parameter<int>("dbscan_min_points", 3);
        declare_parameter<int>("min_cluster_size", 40);
        declare_parameter<double>("semantic_search_radius", 0.2);
        declare_parameter<double>("obstacle_search_radius", 2.0);
        declare_parameter<double>("gain_unknown_radius", 3.0);
        declare_parameter<int>("num_worker_threads", 0);
        declare_parameter<std::string>("distance_metric", "grid_path");
        declare_parameter<int>("path_distance_obstacle_threshold", 50);
        declare_parameter<double>("path_distance_unreachable_m", 1000.0);
        declare_parameter<double>("path_distance_query_radius_m", 0.75);
        declare_parameter<bool>("hard_frontier_filter_enabled", true);
        declare_parameter<double>("frontier_obstacle_clearance_m", 0.30);
        declare_parameter<double>("frontier_representative_clearance_radius_m", 1.20);
        declare_parameter<int>("frontier_occupied_threshold", 50);
        declare_parameter<double>("stable_frontier_match_radius_m", 1.50);
        declare_parameter<int>("stable_frontier_ttl_cycles", 10);
        declare_parameter<std::vector<double>>(
            "frontier_exclusion_rectangles", std::vector<double>{});

        declare_parameter<std::vector<std::string>>("robot_base_frames",
            std::vector<std::string>{"robot_0/base_link", "robot_1/base_link", "robot_2/base_link"});
        declare_parameter<std::vector<std::string>>("robot_names",
            std::vector<std::string>{"robot_0", "robot_1", "robot_2"});

        update_rate_ = get_parameter("update_rate").as_double();
        frontier_backend_ = get_parameter("frontier_backend").as_string();
        eps_ = get_parameter("dbscan_eps").as_double();
        min_points_ = get_parameter("dbscan_min_points").as_int();
        min_cluster_size_ = get_parameter("min_cluster_size").as_int();
        semantic_search_radius_ = get_parameter("semantic_search_radius").as_double();
        obstacle_search_radius_ = get_parameter("obstacle_search_radius").as_double();
        gain_unknown_radius_ = get_parameter("gain_unknown_radius").as_double();
        distance_metric_ = get_parameter("distance_metric").as_string();
        path_distance_obstacle_threshold_ = get_parameter("path_distance_obstacle_threshold").as_int();
        path_distance_unreachable_m_ = get_parameter("path_distance_unreachable_m").as_double();
        path_distance_query_radius_m_ = get_parameter("path_distance_query_radius_m").as_double();
        hard_frontier_filter_enabled_ = get_parameter("hard_frontier_filter_enabled").as_bool();
        frontier_obstacle_clearance_m_ = get_parameter("frontier_obstacle_clearance_m").as_double();
        frontier_representative_clearance_radius_m_ = std::max(
            frontier_obstacle_clearance_m_,
            get_parameter("frontier_representative_clearance_radius_m").as_double());
        frontier_occupied_threshold_ = get_parameter("frontier_occupied_threshold").as_int();
        stable_frontier_match_radius_m_ = std::max(
            0.1, get_parameter("stable_frontier_match_radius_m").as_double());
        stable_frontier_ttl_cycles_ = std::max(
            1, static_cast<int>(get_parameter("stable_frontier_ttl_cycles").as_int()));
        frontier_exclusion_rectangles_ =
            get_parameter("frontier_exclusion_rectangles").as_double_array();

        if (frontier_exclusion_rectangles_.size() % 4 != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "frontier_exclusion_rectangles requires groups of "
                "[xmin, xmax, ymin, ymax]; ignoring malformed tail");
            frontier_exclusion_rectangles_.resize(
                frontier_exclusion_rectangles_.size() -
                frontier_exclusion_rectangles_.size() % 4);
        }

        if (frontier_backend_ != "wavefront_grid" &&
            frontier_backend_ != "legacy_dbscan") {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown frontier_backend='%s'; falling back to wavefront_grid",
                frontier_backend_.c_str());
            frontier_backend_ = "wavefront_grid";
        }

        robot_base_frames_ = get_parameter("robot_base_frames").as_string_array();
        robot_names_ = get_parameter("robot_names").as_string_array();

        const int configured_threads = get_parameter("num_worker_threads").as_int();
        const unsigned int hardware_threads = std::max(1u, std::thread::hardware_concurrency());
        num_worker_threads_ = configured_threads > 0
            ? static_cast<unsigned int>(configured_threads)
            : hardware_threads;

        if (robot_base_frames_.size() != robot_names_.size()) {
            RCLCPP_ERROR(this->get_logger(), "robot_base_frames and robot_names must have the same size!");
            throw std::runtime_error("Configuration mismatch");
        }

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_map", 10,
            std::bind(&FrontierDetector::mapCallback, this, std::placeholders::_1));

        semantic_float_sub_ = this->create_subscription<semantic_grid_msgs::msg::SemanticGrid>(
            "/semantic_grid_float", 10,
            std::bind(&FrontierDetector::semanticFloatCallback, this, std::placeholders::_1));

        semantic_risk_sub_ = this->create_subscription<semantic_grid_msgs::msg::SemanticGrid>(
            "/semantic_risk_grid_float", 10,
            std::bind(&FrontierDetector::semanticRiskCallback, this, std::placeholders::_1));

        frontier_pub_ = this->create_publisher<frontier_msgs::msg::FrontierArrayExtended>(
            "/frontiers_info", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontiers_markers", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            std::bind(&FrontierDetector::process, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Frontier backend: %s, exclusion_rectangles=%zu",
            frontier_backend_.c_str(),
            frontier_exclusion_rectangles_.size() / 4);
    }

private:
    struct Point {
        double x, y;
        int gx, gy;
    };

    struct RobotPosition {
        std::string name;
        std::string frame_id;
        double x;
        double y;
        bool valid;
    };

    struct DistanceField {
        bool valid{false};
        int width{0};
        int height{0};
        double resolution{0.0};
        double origin_x{0.0};
        double origin_y{0.0};
        std::vector<double> distance_m;
    };

    struct ClusterComputationResult {
        bool valid{false};
        std::size_t cid{0};
        double cx{0.0};
        double cy{0.0};
        float semantic_score{0.0f};
        float semantic_risk{0.0f};
        double obstacle_density{0.0};
        double risk_level{0.0};
        int unknown_support_cells{0};
        double exploration_gain{0.0};
        std::vector<double> distances;
        std::size_t cluster_size{0};
        std::string distance_str;
    };

    struct FrontierTrack {
        int id{-1};
        double x{0.0};
        double y{0.0};
        std::uint64_t last_seen_cycle{0};
    };

    double computeExplorationGainScore(
        std::size_t cluster_size,
        int unknown_support_cells) const
    {
        if (cluster_size == 0 || unknown_support_cells <= 0) {
            return 0.0;
        }

        // Frontier length alone tends to overvalue long and thin wall-edge clusters.
        // We combine three terms:
        // 1) frontier_term: keeps some preference for stable/large clusters
        // 2) support_term: rewards actual unknown area behind the frontier
        // 3) efficiency_term: penalizes frontiers whose unknown support is tiny
        //    relative to their geometric extent
        const double frontier_cells = static_cast<double>(cluster_size);
        const double frontier_term =
            std::log1p(frontier_cells) / std::log1p(3000.0);

        const double support_cells = static_cast<double>(unknown_support_cells);
        const double support_term =
            std::log1p(support_cells) / std::log1p(12000.0);

        const double support_per_frontier = support_cells / std::max(1.0, frontier_cells);
        const double efficiency_term = std::clamp(support_per_frontier / 18.0, 0.0, 1.0);

        const double blended =
            std::sqrt(std::max(0.0, frontier_term) * std::max(0.0, support_term)) *
            (0.4 + 0.6 * efficiency_term);

        return std::clamp(blended, 0.0, 1.0);
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    semantic_grid_msgs::msg::SemanticGrid::SharedPtr semantic_float_map_;
    semantic_grid_msgs::msg::SemanticGrid::SharedPtr semantic_risk_map_;

    std::vector<std::string> robot_base_frames_;
    std::vector<std::string> robot_names_;
    double update_rate_, eps_, semantic_search_radius_;
    double obstacle_search_radius_;
    double gain_unknown_radius_;
    std::string frontier_backend_{"wavefront_grid"};
    std::string distance_metric_{"grid_path"};
    int path_distance_obstacle_threshold_{50};
    double path_distance_unreachable_m_{1000.0};
    double path_distance_query_radius_m_{0.75};
    bool hard_frontier_filter_enabled_{true};
    double frontier_obstacle_clearance_m_{0.30};
    double frontier_representative_clearance_radius_m_{1.20};
    int frontier_occupied_threshold_{50};
    double stable_frontier_match_radius_m_{1.50};
    int stable_frontier_ttl_cycles_{10};
    std::vector<double> frontier_exclusion_rectangles_;
    int min_points_, min_cluster_size_;
    unsigned int num_worker_threads_{1};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<semantic_grid_msgs::msg::SemanticGrid>::SharedPtr semantic_float_sub_;
    rclcpp::Subscription<semantic_grid_msgs::msg::SemanticGrid>::SharedPtr semantic_risk_sub_;

    rclcpp::Publisher<frontier_msgs::msg::FrontierArrayExtended>::SharedPtr frontier_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<FrontierTrack> frontier_tracks_;
    int next_frontier_id_{0};
    std::uint64_t frontier_cycle_{0};

    std::vector<int> assignStableFrontierIds(
        const std::vector<ClusterComputationResult> &results)
    {
        ++frontier_cycle_;
        std::vector<int> assigned(results.size(), -1);
        std::vector<bool> track_used(frontier_tracks_.size(), false);

        struct Match {
            double distance;
            std::size_t result_index;
            std::size_t track_index;
        };
        std::vector<Match> matches;
        for (std::size_t result_index = 0; result_index < results.size(); ++result_index) {
            if (!results[result_index].valid) continue;
            for (std::size_t track_index = 0; track_index < frontier_tracks_.size(); ++track_index) {
                const auto &track = frontier_tracks_[track_index];
                const double distance = std::hypot(
                    results[result_index].cx - track.x,
                    results[result_index].cy - track.y);
                if (distance <= stable_frontier_match_radius_m_) {
                    matches.push_back({distance, result_index, track_index});
                }
            }
        }
        std::sort(matches.begin(), matches.end(),
            [](const Match &a, const Match &b) { return a.distance < b.distance; });

        for (const auto &match : matches) {
            if (assigned[match.result_index] >= 0 || track_used[match.track_index]) continue;
            auto &track = frontier_tracks_[match.track_index];
            assigned[match.result_index] = track.id;
            track.x = results[match.result_index].cx;
            track.y = results[match.result_index].cy;
            track.last_seen_cycle = frontier_cycle_;
            track_used[match.track_index] = true;
        }

        for (std::size_t result_index = 0; result_index < results.size(); ++result_index) {
            if (!results[result_index].valid || assigned[result_index] >= 0) continue;
            const int id = next_frontier_id_++;
            assigned[result_index] = id;
            frontier_tracks_.push_back({
                id, results[result_index].cx, results[result_index].cy, frontier_cycle_});
        }

        frontier_tracks_.erase(
            std::remove_if(
                frontier_tracks_.begin(), frontier_tracks_.end(),
                [this](const FrontierTrack &track) {
                    return frontier_cycle_ > track.last_seen_cycle +
                        static_cast<std::uint64_t>(stable_frontier_ttl_cycles_);
                }),
            frontier_tracks_.end());
        return assigned;
    }

    double computeObstacleDensity(
        double cx,
        double cy,
        double radius,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map) const
    {
        if (!map) return 0.0;

        const auto &map_info = map->info;
        const double res = map_info.resolution;
        const double ox = map_info.origin.position.x;
        const double oy = map_info.origin.position.y;

        const int gx_center = static_cast<int>((cx - ox) / res);
        const int gy_center = static_cast<int>((cy - oy) / res);
        const int range = static_cast<int>(radius / res) + 1;

        int occupied = 0;
        int free = 0;

        for (int dy = -range; dy <= range; ++dy) {
            for (int dx = -range; dx <= range; ++dx) {
                const int gx = gx_center + dx;
                const int gy = gy_center + dy;
                if (gx < 0 || gx >= static_cast<int>(map_info.width) ||
                    gy < 0 || gy >= static_cast<int>(map_info.height)) {
                    continue;
                }

                const double dist = std::hypot(dx * res, dy * res);
                if (dist > radius) continue;

                const int idx = gy * static_cast<int>(map_info.width) + gx;
                const int8_t val = map->data[idx];
                if (val == 0) {
                    ++free;
                } else if (val > 50) {
                    ++occupied;
                }
            }
        }

        if (occupied + free == 0) return 0.0;
        return static_cast<double>(occupied) / static_cast<double>(occupied + free);
    }

    float querySemanticFloat(
        double wx,
        double wy,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_float_map) const
    {
        (void)map;
        if (!semantic_float_map) return 0.0f;

        const auto &semantic_info = semantic_float_map->info;
        const double res = semantic_info.resolution;
        const double ox = semantic_info.origin.position.x;
        const double oy = semantic_info.origin.position.y;
        if (res <= 0.0 || semantic_info.width == 0 || semantic_info.height == 0) return 0.0f;

        const int gx = static_cast<int>(std::floor((wx - ox) / res));
        const int gy = static_cast<int>(std::floor((wy - oy) / res));

        if (gx >= 0 && gx < static_cast<int>(semantic_info.width) &&
            gy >= 0 && gy < static_cast<int>(semantic_info.height)) {
            const int idx = gy * static_cast<int>(semantic_info.width) + gx;
            if (idx >= 0 && idx < static_cast<int>(semantic_float_map->data.size())) {
                return semantic_float_map->data[idx];
            }
        }
        return 0.0f;
    }

    float querySemanticRisk(
        double wx,
        double wy,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_risk_map) const
    {
        (void)map;
        if (!semantic_risk_map) return 0.0f;

        const auto &semantic_info = semantic_risk_map->info;
        const double res = semantic_info.resolution;
        const double ox = semantic_info.origin.position.x;
        const double oy = semantic_info.origin.position.y;
        if (res <= 0.0 || semantic_info.width == 0 || semantic_info.height == 0) return 0.0f;

        const int gx = static_cast<int>(std::floor((wx - ox) / res));
        const int gy = static_cast<int>(std::floor((wy - oy) / res));

        if (gx >= 0 && gx < static_cast<int>(semantic_info.width) &&
            gy >= 0 && gy < static_cast<int>(semantic_info.height)) {
            const int idx = gy * static_cast<int>(semantic_info.width) + gx;
            if (idx >= 0 && idx < static_cast<int>(semantic_risk_map->data.size())) {
                return semantic_risk_map->data[idx];
            }
        }
        return 0.0f;
    }

    float getClusterSemanticScore(
        const std::vector<Point> &cluster,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_float_map) const
    {
        if (!semantic_float_map || cluster.empty()) return 0.0f;
        double sum = 0.0;
        for (const auto &p : cluster) {
            sum += querySemanticFloat(p.x, p.y, map, semantic_float_map);
        }
        return static_cast<float>(sum / static_cast<double>(cluster.size()));
    }

    float getClusterSemanticRisk(
        const std::vector<Point> &cluster,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_risk_map) const
    {
        if (!semantic_risk_map || cluster.empty()) return 0.0f;
        double sum = 0.0;
        for (const auto &p : cluster) {
            sum += querySemanticRisk(p.x, p.y, map, semantic_risk_map);
        }
        return static_cast<float>(sum / static_cast<double>(cluster.size()));
    }

    int countUnknownSupportCells(
        const std::vector<Point> &cluster,
        double radius,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map) const
    {
        if (!map || cluster.empty()) return 0;

        const auto &map_info = map->info;
        const int width = static_cast<int>(map_info.width);
        const int height = static_cast<int>(map_info.height);
        const double resolution = map_info.resolution;
        const int range = static_cast<int>(std::ceil(radius / resolution));
        const size_t sample_step = std::max<size_t>(1, cluster.size() / 24);
        std::unordered_set<int> unknown_cells;

        for (size_t index = 0; index < cluster.size(); index += sample_step) {
            const auto &point = cluster[index];
            for (int dy = -range; dy <= range; ++dy) {
                for (int dx = -range; dx <= range; ++dx) {
                    const int gx = point.gx + dx;
                    const int gy = point.gy + dy;
                    if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
                        continue;
                    }
                    const double dist = std::hypot(dx * resolution, dy * resolution);
                    if (dist > radius) {
                        continue;
                    }
                    const int map_index = gy * width + gx;
                    if (map->data[map_index] == -1) {
                        unknown_cells.insert(map_index);
                    }
                }
            }
        }

        const auto &last = cluster.back();
        for (int dy = -range; dy <= range; ++dy) {
            for (int dx = -range; dx <= range; ++dx) {
                const int gx = last.gx + dx;
                const int gy = last.gy + dy;
                if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
                    continue;
                }
                const double dist = std::hypot(dx * resolution, dy * resolution);
                if (dist > radius) {
                    continue;
                }
                const int map_index = gy * width + gx;
                if (map->data[map_index] == -1) {
                    unknown_cells.insert(map_index);
                }
            }
        }

        return static_cast<int>(unknown_cells.size());
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
    }

    void semanticFloatCallback(const semantic_grid_msgs::msg::SemanticGrid::SharedPtr msg) {
        semantic_float_map_ = msg;
        static int log_count = 0;
        if (log_count < 3 && msg && !msg->data.empty()) {
            ++log_count;
            float min_val = std::numeric_limits<float>::infinity();
            float max_val = -std::numeric_limits<float>::infinity();
            const std::size_t step = std::max<std::size_t>(1, msg->data.size() / 2000);
            for (std::size_t i = 0; i < msg->data.size(); i += step) {
                min_val = std::min(min_val, msg->data[i]);
                max_val = std::max(max_val, msg->data[i]);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Received semantic_grid_float: frame=%s cells=%zu sampled[min=%.3f, max=%.3f] first=%.3f mid=%.3f",
                msg->header.frame_id.c_str(),
                msg->data.size(),
                min_val,
                max_val,
                msg->data.front(),
                msg->data[msg->data.size() / 2]);
        }
    }

    void semanticRiskCallback(const semantic_grid_msgs::msg::SemanticGrid::SharedPtr msg) {
        semantic_risk_map_ = msg;
    }

    bool isFrontier(int x, int y, int w, int h, const std::vector<int8_t>& data) const {
        const int idx = y * w + x;
        if (data[idx] != 0) return false;

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            const int nx = x + dx[i];
            const int ny = y + dy[i];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                if (data[ny * w + nx] == -1) return true;
            }
        }
        return false;
    }

    bool isWavefrontFrontier(
        int x,
        int y,
        int w,
        int h,
        const std::vector<int8_t> &data) const
    {
        const int idx = y * w + x;
        if (data[idx] != 0) return false;

        // Use 8-neighbour unknown support so diagonal map boundaries remain
        // connected. Goal safety and reachability are checked later.
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h &&
                    data[ny * w + nx] == -1) {
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<std::vector<Point>> collectWavefrontClusters(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const std::vector<RobotPosition> &robot_positions) const
    {
        std::vector<std::vector<Point>> clusters;
        if (!map || map->info.resolution <= 0.0) return clusters;

        const int w = static_cast<int>(map->info.width);
        const int h = static_cast<int>(map->info.height);
        if (w <= 0 || h <= 0) return clusters;

        const auto &data = map->data;
        const double res = map->info.resolution;
        const double ox = map->info.origin.position.x;
        const double oy = map->info.origin.position.y;
        const std::size_t cell_count = static_cast<std::size_t>(w) * h;
        if (data.size() < cell_count) {
            RCLCPP_WARN(
                this->get_logger(),
                "OccupancyGrid data is shorter than width*height (%zu < %zu)",
                data.size(), cell_count);
            return clusters;
        }
        std::vector<uint8_t> visited(cell_count, 0);
        std::vector<uint8_t> frontier_mask(cell_count, 0);
        std::queue<int> open;

        // Multi-source WFD: explore known free space reachable from any robot.
        // If a robot lies on a non-traversable map cell, use the same bounded
        // nearest-free fallback as the path-distance calculation.
        const int seed_search_radius = std::max(
            1, static_cast<int>(std::ceil(path_distance_query_radius_m_ / res)));
        for (const auto &robot : robot_positions) {
            if (!robot.valid) continue;
            int gx = 0;
            int gy = 0;
            if (!worldToGrid(map, robot.x, robot.y, gx, gy)) continue;
            int seed_x = gx;
            int seed_y = gy;
            if (!findNearestTraversableCell(
                    map, gx, gy, seed_search_radius, seed_x, seed_y)) {
                continue;
            }
            const int seed_idx = seed_y * w + seed_x;
            if (!visited[seed_idx]) {
                visited[seed_idx] = 1;
                open.push(seed_idx);
            }
        }

        static constexpr int kCardinalDx[4] = {1, -1, 0, 0};
        static constexpr int kCardinalDy[4] = {0, 0, 1, -1};
        while (!open.empty()) {
            const int idx = open.front();
            open.pop();
            const int x = idx % w;
            const int y = idx / w;

            if (isWavefrontFrontier(x, y, w, h, data)) {
                frontier_mask[idx] = 1;
            }

            // Four-connected traversal avoids passing diagonally through the
            // corner of two occupied cells.
            for (int direction = 0; direction < 4; ++direction) {
                const int nx = x + kCardinalDx[direction];
                const int ny = y + kCardinalDy[direction];
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                const int next_idx = ny * w + nx;
                if (visited[next_idx] || !isTraversableCell(map, nx, ny)) continue;
                visited[next_idx] = 1;
                open.push(next_idx);
            }
        }

        // The points already live on a regular grid, so 8-connected component
        // extraction is deterministic and linear, unlike the legacy O(N^2)
        // all-pairs DBSCAN implementation.
        std::vector<uint8_t> clustered(cell_count, 0);
        for (int start = 0; start < static_cast<int>(cell_count); ++start) {
            if (!frontier_mask[start] || clustered[start]) continue;
            std::vector<Point> cluster;
            std::queue<int> frontier_open;
            clustered[start] = 1;
            frontier_open.push(start);

            while (!frontier_open.empty()) {
                const int idx = frontier_open.front();
                frontier_open.pop();
                const int x = idx % w;
                const int y = idx / w;
                cluster.push_back({
                    ox + (x + 0.5) * res,
                    oy + (y + 0.5) * res,
                    x,
                    y});

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = x + dx;
                        const int ny = y + dy;
                        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                        const int next_idx = ny * w + nx;
                        if (!frontier_mask[next_idx] || clustered[next_idx]) continue;
                        clustered[next_idx] = 1;
                        frontier_open.push(next_idx);
                    }
                }
            }
            clusters.push_back(std::move(cluster));
        }
        return clusters;
    }

    std::vector<Point> collectFrontierPointsParallel(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map) const
    {
        std::vector<Point> frontier_pts;
        if (!map) return frontier_pts;

        const int w = static_cast<int>(map->info.width);
        const int h = static_cast<int>(map->info.height);
        const auto &data = map->data;
        const double res = map->info.resolution;
        const double ox = map->info.origin.position.x;
        const double oy = map->info.origin.position.y;

        const unsigned int worker_count = std::max(
            1u,
            std::min<unsigned int>(num_worker_threads_, static_cast<unsigned int>(std::max(1, h))));

        if (worker_count == 1 || h < 32) {
            frontier_pts.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 16);
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    if (isFrontier(x, y, w, h, data)) {
                        const double wx = ox + (x + 0.5) * res;
                        const double wy = oy + (y + 0.5) * res;
                        frontier_pts.push_back({wx, wy, x, y});
                    }
                }
            }
            return frontier_pts;
        }

        std::vector<std::future<std::vector<Point>>> futures;
        futures.reserve(worker_count);

        const int rows_per_worker = std::max(1, h / static_cast<int>(worker_count));
        for (unsigned int worker = 0; worker < worker_count; ++worker) {
            const int y_begin = static_cast<int>(worker) * rows_per_worker;
            const int y_end = (worker == worker_count - 1)
                ? h
                : std::min(h, y_begin + rows_per_worker);

            futures.emplace_back(std::async(std::launch::async,
                [this, &data, w, h, res, ox, oy, y_begin, y_end]() {
                    std::vector<Point> local_frontiers;
                    local_frontiers.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(y_end - y_begin) / 16);
                    for (int y = y_begin; y < y_end; ++y) {
                        for (int x = 0; x < w; ++x) {
                            if (isFrontier(x, y, w, h, data)) {
                                const double wx = ox + (x + 0.5) * res;
                                const double wy = oy + (y + 0.5) * res;
                                local_frontiers.push_back({wx, wy, x, y});
                            }
                        }
                    }
                    return local_frontiers;
                }));
        }

        std::size_t total_size = 0;
        std::vector<std::vector<Point>> chunks;
        chunks.reserve(futures.size());
        for (auto &future : futures) {
            chunks.push_back(future.get());
            total_size += chunks.back().size();
        }

        frontier_pts.reserve(total_size);
        for (auto &chunk : chunks) {
            frontier_pts.insert(frontier_pts.end(), chunk.begin(), chunk.end());
        }

        return frontier_pts;
    }

    std::vector<int> dbscan(const std::vector<Point> &pts) const {
        const int n = static_cast<int>(pts.size());
        std::vector<int> labels(n, -1);
        int cluster_id = 0;

        auto dist = [&](int i, int j) {
            const double dx = pts[i].x - pts[j].x;
            const double dy = pts[i].y - pts[j].y;
            return std::sqrt(dx * dx + dy * dy);
        };

        for (int i = 0; i < n; ++i) {
            if (labels[i] != -1) continue;

            std::vector<int> neighbors;
            for (int j = 0; j < n; ++j) {
                if (dist(i, j) <= eps_) {
                    neighbors.push_back(j);
                }
            }

            if (neighbors.size() < static_cast<std::size_t>(min_points_)) {
                labels[i] = -2;
                continue;
            }

            labels[i] = cluster_id;
            std::queue<int> q;
            for (int nb : neighbors) q.push(nb);

            while (!q.empty()) {
                const int p = q.front();
                q.pop();

                if (labels[p] == -2) labels[p] = cluster_id;
                if (labels[p] != -1) continue;

                labels[p] = cluster_id;

                std::vector<int> neighbors2;
                for (int j = 0; j < n; ++j) {
                    if (dist(p, j) <= eps_) {
                        neighbors2.push_back(j);
                    }
                }

                if (neighbors2.size() >= static_cast<std::size_t>(min_points_)) {
                    for (int nb : neighbors2) q.push(nb);
                }
            }
            ++cluster_id;
        }
        return labels;
    }

    std::vector<RobotPosition> getAllRobotPositions() {
        std::vector<RobotPosition> robot_positions;

        for (std::size_t i = 0; i < robot_base_frames_.size(); ++i) {
            const auto &base_frame = robot_base_frames_[i];
            const auto &robot_name = robot_names_[i];

            try {
                geometry_msgs::msg::TransformStamped transform;
                transform = tf_buffer_.lookupTransform(
                    "global_map",
                    base_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1));

                robot_positions.push_back({
                    robot_name,
                    base_frame,
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    true
                });

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Failed to get transform for robot %s (%s): %s",
                    robot_name.c_str(), base_frame.c_str(), ex.what());

                robot_positions.push_back({
                    robot_name,
                    base_frame,
                    0.0,
                    0.0,
                    false
                });
            }
        }

        return robot_positions;
    }

    bool worldToGrid(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        double wx,
        double wy,
        int &gx,
        int &gy) const
    {
        if (!map || map->info.resolution <= 0.0) return false;
        gx = static_cast<int>(std::floor((wx - map->info.origin.position.x) / map->info.resolution));
        gy = static_cast<int>(std::floor((wy - map->info.origin.position.y) / map->info.resolution));
        return gx >= 0 && gx < static_cast<int>(map->info.width) &&
               gy >= 0 && gy < static_cast<int>(map->info.height);
    }

    bool isTraversableCell(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        int gx,
        int gy) const
    {
        if (!map) return false;
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) return false;

        const int idx = gy * width + gx;
        const int8_t val = map->data[idx];
        return val >= 0 && val <= path_distance_obstacle_threshold_;
    }

    bool hasObstacleClearance(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        int gx,
        int gy) const
    {
        if (!map || map->info.resolution <= 0.0) return false;
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) return false;

        const int center_idx = gy * width + gx;
        // A published goal must itself be a known free cell. Do not accept
        // unknown cells or low-probability occupied cells as a goal position.
        if (map->data[center_idx] != 0) return false;

        const double resolution = map->info.resolution;
        const int radius_cells = std::max(
            0, static_cast<int>(std::ceil(frontier_obstacle_clearance_m_ / resolution)));

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (std::hypot(dx * resolution, dy * resolution) >
                    frontier_obstacle_clearance_m_) {
                    continue;
                }
                const int nx = gx + dx;
                const int ny = gy + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    return false;
                }
                const int8_t value = map->data[ny * width + nx];
                if (value >= frontier_occupied_threshold_) {
                    return false;
                }
            }
        }
        return true;
    }

    bool selectSafeClusterRepresentative(
        const std::vector<Point> &cluster,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        double mean_x,
        double mean_y,
        double &out_x,
        double &out_y) const
    {
        if (cluster.empty() || !map) return false;

        const Point *best = nullptr;
        double best_clearance = -1.0;
        double best_distance_sq = std::numeric_limits<double>::infinity();
        for (const auto &point : cluster) {
            if (!hasObstacleClearance(map, point.gx, point.gy)) continue;
            // Do not reject the whole cluster when its centroid-side candidate
            // overlaps a known simulator wall. Skip only that candidate so a
            // safe point on another part of the same unknown boundary can be
            // selected instead.
            if (isInsideFrontierExclusionRectangle(point.x, point.y)) continue;
            const double clearance = nearestOccupiedDistance(
                map,
                point.gx,
                point.gy,
                frontier_representative_clearance_radius_m_);
            const double dx = point.x - mean_x;
            const double dy = point.y - mean_y;
            const double distance_sq = dx * dx + dy * dy;
            // A large connected frontier may contain both a wall artefact and
            // a genuine open unknown boundary. Prefer the candidate with the
            // greatest measured wall clearance; use centroid distance only as
            // a deterministic tie-breaker.
            if (clearance > best_clearance + 1e-6 ||
                (std::abs(clearance - best_clearance) <= 1e-6 &&
                 distance_sq < best_distance_sq)) {
                best_clearance = clearance;
                best_distance_sq = distance_sq;
                best = &point;
            }
        }

        if (!best) return false;
        out_x = best->x;
        out_y = best->y;
        return true;
    }

    double nearestOccupiedDistance(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        int gx,
        int gy,
        double max_radius_m) const
    {
        if (!map || map->info.resolution <= 0.0 || max_radius_m <= 0.0) {
            return 0.0;
        }
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const double resolution = map->info.resolution;
        const int radius_cells = std::max(
            1, static_cast<int>(std::ceil(max_radius_m / resolution)));
        double nearest = max_radius_m;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const double distance = std::hypot(dx * resolution, dy * resolution);
                if (distance > max_radius_m || distance >= nearest) continue;
                const int nx = gx + dx;
                const int ny = gy + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                if (map->data[ny * width + nx] >= frontier_occupied_threshold_) {
                    nearest = distance;
                }
            }
        }
        return nearest;
    }

    bool isInsideFrontierExclusionRectangle(double x, double y) const
    {
        for (std::size_t i = 0; i + 3 < frontier_exclusion_rectangles_.size(); i += 4) {
            const double xmin = std::min(
                frontier_exclusion_rectangles_[i],
                frontier_exclusion_rectangles_[i + 1]);
            const double xmax = std::max(
                frontier_exclusion_rectangles_[i],
                frontier_exclusion_rectangles_[i + 1]);
            const double ymin = std::min(
                frontier_exclusion_rectangles_[i + 2],
                frontier_exclusion_rectangles_[i + 3]);
            const double ymax = std::max(
                frontier_exclusion_rectangles_[i + 2],
                frontier_exclusion_rectangles_[i + 3]);
            if (x >= xmin && x <= xmax && y >= ymin && y <= ymax) {
                return true;
            }
        }
        return false;
    }

    bool findNearestTraversableCell(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        int start_x,
        int start_y,
        int max_radius_cells,
        int &out_x,
        int &out_y) const
    {
        if (isTraversableCell(map, start_x, start_y)) {
            out_x = start_x;
            out_y = start_y;
            return true;
        }

        for (int radius = 1; radius <= max_radius_cells; ++radius) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != radius) continue;
                    const int gx = start_x + dx;
                    const int gy = start_y + dy;
                    if (isTraversableCell(map, gx, gy)) {
                        out_x = gx;
                        out_y = gy;
                        return true;
                    }
                }
            }
        }

        return false;
    }

    DistanceField buildDistanceFieldForRobot(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const RobotPosition &robot) const
    {
        DistanceField field;
        if (!map || !robot.valid || map->info.resolution <= 0.0 ||
            map->info.width == 0 || map->info.height == 0) {
            return field;
        }

        field.valid = true;
        field.width = static_cast<int>(map->info.width);
        field.height = static_cast<int>(map->info.height);
        field.resolution = map->info.resolution;
        field.origin_x = map->info.origin.position.x;
        field.origin_y = map->info.origin.position.y;
        field.distance_m.assign(
            static_cast<std::size_t>(field.width) * static_cast<std::size_t>(field.height),
            std::numeric_limits<double>::infinity());

        int robot_gx = 0;
        int robot_gy = 0;
        if (!worldToGrid(map, robot.x, robot.y, robot_gx, robot_gy)) {
            field.valid = false;
            return field;
        }

        const int start_search_radius_cells =
            std::max(1, static_cast<int>(std::ceil(path_distance_query_radius_m_ / field.resolution)));
        int start_x = robot_gx;
        int start_y = robot_gy;
        if (!findNearestTraversableCell(map, robot_gx, robot_gy, start_search_radius_cells, start_x, start_y)) {
            field.valid = false;
            return field;
        }

        using QueueItem = std::pair<double, int>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

        const int start_idx = start_y * field.width + start_x;
        field.distance_m[start_idx] = 0.0;
        open.emplace(0.0, start_idx);

        const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!open.empty()) {
            const auto [cur_dist, idx] = open.top();
            open.pop();
            if (cur_dist > field.distance_m[idx]) continue;

            const int x = idx % field.width;
            const int y = idx / field.width;

            for (int i = 0; i < 8; ++i) {
                const int nx = x + dxs[i];
                const int ny = y + dys[i];
                if (!isTraversableCell(map, nx, ny)) continue;

                if (dxs[i] != 0 && dys[i] != 0) {
                    // Avoid cutting diagonally through obstacle corners.
                    if (!isTraversableCell(map, x + dxs[i], y) ||
                        !isTraversableCell(map, x, y + dys[i])) {
                        continue;
                    }
                }

                const int nidx = ny * field.width + nx;
                const double step = field.resolution * ((dxs[i] != 0 && dys[i] != 0) ? std::sqrt(2.0) : 1.0);
                const double nd = cur_dist + step;
                if (nd < field.distance_m[nidx]) {
                    field.distance_m[nidx] = nd;
                    open.emplace(nd, nidx);
                }
            }
        }

        return field;
    }

    std::vector<DistanceField> buildDistanceFields(
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const std::vector<RobotPosition> &robot_positions) const
    {
        std::vector<DistanceField> fields;
        fields.reserve(robot_positions.size());
        if (distance_metric_ != "grid_path") {
            return fields;
        }

        for (const auto &robot : robot_positions) {
            fields.push_back(buildDistanceFieldForRobot(map, robot));
        }

        return fields;
    }

    double queryPathDistanceAtWorld(
        const DistanceField &field,
        double wx,
        double wy) const
    {
        if (!field.valid || field.resolution <= 0.0 ||
            field.distance_m.empty()) {
            return path_distance_unreachable_m_;
        }

        const int gx = static_cast<int>(std::floor((wx - field.origin_x) / field.resolution));
        const int gy = static_cast<int>(std::floor((wy - field.origin_y) / field.resolution));
        if (gx < 0 || gx >= field.width || gy < 0 || gy >= field.height) {
            return path_distance_unreachable_m_;
        }

        const int center_idx = gy * field.width + gx;
        if (std::isfinite(field.distance_m[center_idx])) {
            return field.distance_m[center_idx];
        }

        const int range = std::max(1, static_cast<int>(std::ceil(path_distance_query_radius_m_ / field.resolution)));
        double best = std::numeric_limits<double>::infinity();
        for (int dy = -range; dy <= range; ++dy) {
            for (int dx = -range; dx <= range; ++dx) {
                const int nx = gx + dx;
                const int ny = gy + dy;
                if (nx < 0 || nx >= field.width || ny < 0 || ny >= field.height) continue;

                const double offset_m = std::hypot(dx * field.resolution, dy * field.resolution);
                if (offset_m > path_distance_query_radius_m_) continue;

                const int idx = ny * field.width + nx;
                const double d = field.distance_m[idx];
                if (std::isfinite(d)) {
                    best = std::min(best, d + offset_m);
                }
            }
        }

        return std::isfinite(best) ? best : path_distance_unreachable_m_;
    }

    std::vector<double> calculateDistancesToRobots(
        double cx,
        double cy,
        const std::vector<RobotPosition> &robot_positions,
        const std::vector<DistanceField> &distance_fields) const
    {
        std::vector<double> distances;
        distances.reserve(robot_positions.size());

        for (std::size_t i = 0; i < robot_positions.size(); ++i) {
            const auto &robot = robot_positions[i];
            if (robot.valid) {
                if (distance_metric_ == "grid_path" && i < distance_fields.size() && distance_fields[i].valid) {
                    distances.push_back(queryPathDistanceAtWorld(distance_fields[i], cx, cy));
                } else {
                    distances.push_back(std::hypot(cx - robot.x, cy - robot.y));
                }
            } else {
                distances.push_back(-1.0);
            }
        }

        return distances;
    }

    ClusterComputationResult computeClusterResult(
        std::size_t cid,
        const std::vector<Point> &cluster,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_float_map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_risk_map,
        const std::vector<RobotPosition> &robot_positions,
        const std::vector<DistanceField> &distance_fields) const
    {
        ClusterComputationResult result;
        result.cid = cid;
        result.cluster_size = cluster.size();

        if (cluster.size() < static_cast<std::size_t>(min_cluster_size_)) return result;
        if (cluster.size() < static_cast<std::size_t>(min_points_)) return result;
        if (!map) return result;

        double sx = 0.0;
        double sy = 0.0;
        for (const auto &p : cluster) {
            sx += p.x;
            sy += p.y;
        }
        const double mean_x = sx / static_cast<double>(cluster.size());
        const double mean_y = sy / static_cast<double>(cluster.size());
        result.cx = mean_x;
        result.cy = mean_y;

        if (hard_frontier_filter_enabled_ &&
            !selectSafeClusterRepresentative(
                cluster, map, mean_x, mean_y, result.cx, result.cy)) {
            return result;
        }

        result.semantic_score = getClusterSemanticScore(cluster, map, semantic_float_map);
        result.semantic_risk = getClusterSemanticRisk(cluster, map, semantic_risk_map);
        result.obstacle_density = computeObstacleDensity(result.cx, result.cy, obstacle_search_radius_, map);
        result.risk_level = 0.5 * result.obstacle_density + 0.5 * result.semantic_risk;
        result.unknown_support_cells = countUnknownSupportCells(cluster, gain_unknown_radius_, map);

        result.exploration_gain =
            computeExplorationGainScore(result.cluster_size, result.unknown_support_cells);
        result.distances = calculateDistancesToRobots(result.cx, result.cy, robot_positions, distance_fields);

        // At least one robot must have an actual grid path to the selected
        // frontier cell. This prevents the query-radius fallback from keeping
        // isolated wall-side clusters alive forever.
        if (hard_frontier_filter_enabled_) {
            const bool reachable = std::any_of(
                result.distances.begin(), result.distances.end(),
                [this](double distance) {
                    return distance >= 0.0 && distance < path_distance_unreachable_m_;
                });
            if (!reachable) return result;
        }

        std::ostringstream oss;
        for (std::size_t i = 0; i < result.distances.size(); ++i) {
            oss << robot_names_[i] << ": ";
            if (result.distances[i] >= 0.0) {
                oss << result.distances[i];
            } else {
                oss << "N/A";
            }
            oss << "m ";
        }
        result.distance_str = oss.str();
        result.valid = true;
        return result;
    }

    std::vector<ClusterComputationResult> computeClustersParallel(
        const std::vector<std::vector<Point>> &clusters,
        const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_float_map,
        const semantic_grid_msgs::msg::SemanticGrid::SharedPtr &semantic_risk_map,
        const std::vector<RobotPosition> &robot_positions,
        const std::vector<DistanceField> &distance_fields) const
    {
        std::vector<ClusterComputationResult> results(clusters.size());
        if (clusters.empty()) return results;

        const unsigned int worker_count = std::max(
            1u,
            std::min<unsigned int>(num_worker_threads_, static_cast<unsigned int>(clusters.size())));

        if (worker_count == 1 || clusters.size() < 4) {
            for (std::size_t cid = 0; cid < clusters.size(); ++cid) {
                results[cid] = computeClusterResult(
                    cid, clusters[cid], map, semantic_float_map, semantic_risk_map, robot_positions, distance_fields);
            }
            return results;
        }

        std::vector<std::future<void>> futures;
        futures.reserve(worker_count);

        const std::size_t chunk_size = std::max<std::size_t>(1, clusters.size() / worker_count);
        for (unsigned int worker = 0; worker < worker_count; ++worker) {
            const std::size_t begin = static_cast<std::size_t>(worker) * chunk_size;
            if (begin >= clusters.size()) break;
            const std::size_t end = (worker == worker_count - 1)
                ? clusters.size()
                : std::min(clusters.size(), begin + chunk_size);

            futures.emplace_back(std::async(std::launch::async,
                [this, &clusters, &results, &map, &semantic_float_map, &semantic_risk_map, &robot_positions, &distance_fields, begin, end]() {
                    for (std::size_t cid = begin; cid < end; ++cid) {
                        results[cid] = computeClusterResult(
                            cid, clusters[cid], map, semantic_float_map, semantic_risk_map, robot_positions, distance_fields);
                    }
                }));
        }

        for (auto &future : futures) {
            future.get();
        }

        return results;
    }

    void addDeleteAllMarker(visualization_msgs::msg::MarkerArray &marker_array) const
    {
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);
    }

    visualization_msgs::msg::Marker makeClusterPointsMarker(
        int id,
        const std::vector<Point> &cluster,
        float semantic_score) const
    {
        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "global_map";
        mk.header.stamp = now();
        mk.ns = "frontier_points";
        mk.id = id;
        mk.type = visualization_msgs::msg::Marker::POINTS;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 0.06;
        mk.scale.y = 0.06;
        mk.color.a = 1.0;

        const float norm_score = std::clamp((semantic_score + 0.5f) / 1.5f, 0.0f, 1.0f);
        mk.color.r = norm_score;
        mk.color.g = 1.0f - norm_score;
        mk.color.b = 0.0f;

        mk.points.reserve(cluster.size());
        for (const auto &p : cluster) {
            geometry_msgs::msg::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.02;
            mk.points.push_back(pt);
        }
        return mk;
    }

    visualization_msgs::msg::Marker makeClusterCenterMarker(
        int id,
        double cx,
        double cy) const
    {
        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "global_map";
        mk.header.stamp = now();
        mk.ns = "frontier_centers";
        mk.id = id;
        mk.type = visualization_msgs::msg::Marker::SPHERE;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose.position.x = cx;
        mk.pose.position.y = cy;
        mk.pose.position.z = 0.08;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 0.18;
        mk.scale.y = 0.18;
        mk.scale.z = 0.18;
        mk.color.r = 1.0f;
        mk.color.g = 0.1f;
        mk.color.b = 0.1f;
        mk.color.a = 0.95f;
        return mk;
    }

    visualization_msgs::msg::Marker makeClusterTextMarker(
        int id,
        double cx,
        double cy,
        int frontier_id,
        std::size_t cluster_size) const
    {
        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "global_map";
        mk.header.stamp = now();
        mk.ns = "frontier_labels";
        mk.id = id;
        mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose.position.x = cx;
        mk.pose.position.y = cy;
        mk.pose.position.z = 0.35;
        mk.pose.orientation.w = 1.0;
        mk.scale.z = 0.18;
        mk.color.r = 1.0f;
        mk.color.g = 1.0f;
        mk.color.b = 1.0f;
        mk.color.a = 1.0f;
        mk.text = "id=" + std::to_string(frontier_id) + " n=" + std::to_string(cluster_size);
        return mk;
    }

    void process() {
        const auto start_time = std::chrono::steady_clock::now();
        const auto map = map_;
        const auto semantic_float_map = semantic_float_map_;
        const auto semantic_risk_map = semantic_risk_map_;

        if (!map) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No global map received yet");
            return;
        }

        auto robot_positions = getAllRobotPositions();

        bool any_valid_robot = false;
        for (const auto &robot : robot_positions) {
            if (robot.valid) {
                any_valid_robot = true;
                break;
            }
        }

        if (!any_valid_robot) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No valid robot positions available");
            return;
        }
        const auto distance_fields = buildDistanceFields(map, robot_positions);

        visualization_msgs::msg::MarkerArray marker_array;
        addDeleteAllMarker(marker_array);

        std::vector<std::vector<Point>> clusters;
        std::size_t frontier_point_count = 0;
        if (frontier_backend_ == "legacy_dbscan") {
            const std::vector<Point> frontier_pts = collectFrontierPointsParallel(map);
            frontier_point_count = frontier_pts.size();
            if (!frontier_pts.empty()) {
                const std::vector<int> labels = dbscan(frontier_pts);
                const int max_cluster_id = *std::max_element(labels.begin(), labels.end());
                if (max_cluster_id >= 0) {
                    clusters.resize(static_cast<std::size_t>(max_cluster_id) + 1);
                    for (std::size_t i = 0; i < labels.size(); ++i) {
                        if (labels[i] >= 0) {
                            clusters[static_cast<std::size_t>(labels[i])].push_back(frontier_pts[i]);
                        }
                    }
                }
            }
        } else {
            clusters = collectWavefrontClusters(map, robot_positions);
            for (const auto &cluster : clusters) {
                frontier_point_count += cluster.size();
            }
        }

        if (clusters.empty()) {
            marker_pub_->publish(marker_array);
            return;
        }

        const auto cluster_results = computeClustersParallel(
            clusters, map, semantic_float_map, semantic_risk_map, robot_positions, distance_fields);
        const auto stable_frontier_ids = assignStableFrontierIds(cluster_results);

        frontier_msgs::msg::FrontierArrayExtended fa;
        fa.header.frame_id = "global_map";
        fa.header.stamp = now();

        int valid_cluster_count = 0;
        for (std::size_t cid = 0; cid < clusters.size(); ++cid) {
            const auto &cl = clusters[cid];
            const auto &result = cluster_results[cid];
            if (!result.valid) continue;
            const int stable_id = stable_frontier_ids[cid];

            marker_array.markers.push_back(
                makeClusterPointsMarker(stable_id, cl, result.semantic_score));
            marker_array.markers.push_back(
                makeClusterCenterMarker(stable_id, result.cx, result.cy));
            marker_array.markers.push_back(
                makeClusterTextMarker(stable_id, result.cx, result.cy,
                    stable_id, cl.size()));

            frontier_msgs::msg::FrontierInfoExtended fi;
            fi.id = stable_id;
            fi.position[0] = result.cx;
            fi.position[1] = result.cy;
            fi.semantic_score = result.semantic_score;
            fi.distances_to_robots = result.distances;
            fi.robot_names = robot_names_;
            fi.risk_level = result.risk_level;
            fi.exploration_gain = result.exploration_gain;
            fa.frontiers_extended.push_back(fi);
            ++valid_cluster_count;

            RCLCPP_INFO(
                this->get_logger(),
                "Frontier %d: center (%.2f, %.2f), semantic score: %.3f, semantic risk: %.3f, obstacle density: %.3f, risk_level: %.3f, cluster_size=%zu, unknown_support=%d, exploration_gain: %.3f, distances: %s",
                stable_id,
                result.cx,
                result.cy,
                result.semantic_score,
                result.semantic_risk,
                result.obstacle_density,
                result.risk_level,
                cl.size(),
                result.unknown_support_cells,
                result.exploration_gain,
                result.distance_str.c_str());

        }

        marker_pub_->publish(marker_array);

        if (!fa.frontiers_extended.empty()) {
            frontier_pub_->publish(fa);
        } else {
        }

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed_ms >= 250.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "[PERF/FRONTIER] process slow: %.1f ms, frontier_pts=%zu, clusters=%zu, valid_clusters=%d",
                elapsed_ms, frontier_point_count, clusters.size(), valid_cluster_count);
        } else {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "[PERF/FRONTIER] process: %.1f ms, frontier_pts=%zu, clusters=%zu, valid_clusters=%d",
                elapsed_ms, frontier_point_count, clusters.size(), valid_cluster_count);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDetector>());
    rclcpp::shutdown();
    return 0;
}
