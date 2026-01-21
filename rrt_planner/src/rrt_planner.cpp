
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

// ============================================================================
// RRT Node Structure
// ============================================================================
struct RRTNode
{
    double x, y;
    std::shared_ptr<RRTNode> parent;
    RRTNode(double x_, double y_) : x(x_), y(y_), parent(nullptr) {}
};

// ============================================================================
// RRT Planner
// ============================================================================
class RRTPlanner
{
public:
    RRTPlanner(const nav_msgs::msg::OccupancyGrid& grid, 
               int max_iter = 5000, 
               double step = 0.5, 
               double bias = 0.1,
               double robot_radius = 0.105,  // Half of Turtlebot3 width (0.178/2 + margin)
               double inflation_radius = 0.15) // Additional safety margin
        : grid_(grid), max_iterations_(max_iter), step_size_(step), 
          goal_bias_(bias), robot_radius_(robot_radius),
          inflation_radius_(inflation_radius), uniform_dist_(0.0, 1.0)
    {
        width_ = grid_.info.width;
        height_ = grid_.info.height;
        resolution_ = grid_.info.resolution;
        origin_x_ = grid_.info.origin.position.x;
        origin_y_ = grid_.info.origin.position.y;
        grid_data_ = grid_.data;
        
        // Calculate inflation in grid cells
        inflation_cells_ = static_cast<int>(std::ceil(
            (robot_radius_ + inflation_radius_) / resolution_));
        
        rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        
        std::cout << "RRT Planner initialized with:" << std::endl;
        std::cout << "  Robot radius: " << robot_radius_ << "m" << std::endl;
        std::cout << "  Inflation radius: " << inflation_radius_ << "m" << std::endl;
        std::cout << "  Total safety margin: " << (robot_radius_ + inflation_radius_) << "m" << std::endl;
        std::cout << "  Inflation cells: " << inflation_cells_ << std::endl;
    }
    
    nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped& start,
                             const geometry_msgs::msg::PoseStamped& goal)
    {
        double sx = start.pose.position.x, sy = start.pose.position.y;
        double gx = goal.pose.position.x, gy = goal.pose.position.y;
        
        if (!isValidWithInflation(sx, sy)) {
            throw std::runtime_error("Start position too close to obstacles!");
        }
        if (!isValidWithInflation(gx, gy)) {
            throw std::runtime_error("Goal position too close to obstacles!");
        }
        
        nodes_.clear();
        nodes_.push_back(std::make_shared<RRTNode>(sx, sy));
        
        for (int i = 0; i < max_iterations_; ++i) {
            double rx, ry;
            if (uniform_dist_(rng_) < goal_bias_) {
                rx = gx; ry = gy;
            } else {
                rx = uniform_dist_(rng_) * width_ * resolution_ + origin_x_;
                ry = uniform_dist_(rng_) * height_ * resolution_ + origin_y_;
            }
            
            auto nearest = getNearestNode(rx, ry);
            auto [nx, ny] = steer(nearest, rx, ry);
            
            if (isValidWithInflation(nx, ny) && 
                isCollisionFreeWithInflation(nearest->x, nearest->y, nx, ny)) {
                auto new_node = std::make_shared<RRTNode>(nx, ny);
                new_node->parent = nearest;
                nodes_.push_back(new_node);
                
                double dist = distance(nx, ny, gx, gy);
                if (dist < step_size_ && 
                    isCollisionFreeWithInflation(nx, ny, gx, gy)) {
                    auto goal_node = std::make_shared<RRTNode>(gx, gy);
                    goal_node->parent = new_node;
                    nodes_.push_back(goal_node);
                    return extractPath(goal_node, start.header.frame_id);
                }
            }
        }
        throw std::runtime_error("No path found after max iterations!");
    }
    
    visualization_msgs::msg::MarkerArray getTreeViz(const std::string& frame) const
    {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker line;
        line.header.frame_id = frame;
        line.header.stamp = rclcpp::Clock().now();
        line.ns = "rrt_tree";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.scale.x = 0.1;
        line.color.r = 0.0;
        line.color.g = 1.0;
        line.color.b = 0.0;
        line.color.a = 0.4;
        
        for (const auto& node : nodes_) {
            if (node->parent) {
                geometry_msgs::msg::Point p1, p2;
                p1.x = node->parent->x; p1.y = node->parent->y;
                p2.x = node->x; p2.y = node->y;
                line.points.push_back(p1);
                line.points.push_back(p2);
            }
        }
        markers.markers.push_back(line);
        return markers;
    }

private:
    nav_msgs::msg::OccupancyGrid grid_;
    std::vector<int8_t> grid_data_;
    int width_, height_, max_iterations_;
    double resolution_, origin_x_, origin_y_, step_size_, goal_bias_;
    double robot_radius_, inflation_radius_;
    int inflation_cells_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;
    std::vector<std::shared_ptr<RRTNode>> nodes_;
    
    // Check if position is valid considering robot size and inflation
    bool isValidWithInflation(double x, double y) const {
        int gx = (x - origin_x_) / resolution_;
        int gy = (y - origin_y_) / resolution_;
        
        // Check all cells within robot radius + inflation
        for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
            for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
                int check_x = gx + dx;
                int check_y = gy + dy;
                
                // Check if within circular footprint
                double dist = sqrt(dx*dx + dy*dy) * resolution_;
                if (dist > (robot_radius_ + inflation_radius_)) continue;
                
                // Check bounds
                if (check_x < 0 || check_x >= width_ || 
                    check_y < 0 || check_y >= height_) {
                    return false;
                }
                
                // Check occupancy
                if (grid_data_[check_y * width_ + check_x] > 50) {
                    return false;
                }
            }
        }
        return true;
    }
    
    bool isCollisionFreeWithInflation(double x1, double y1, double x2, double y2) const {
        double dist = distance(x1, y1, x2, y2);
        int steps = dist / (resolution_ * 0.5) + 1;
        
        for (int i = 0; i <= steps; ++i) {
            double x = x1 + (x2 - x1) * i / steps;
            double y = y1 + (y2 - y1) * i / steps;
            if (!isValidWithInflation(x, y)) return false;
        }
        return true;
    }
    
    std::shared_ptr<RRTNode> getNearestNode(double x, double y) const {
        double min_dist = 1e9;
        std::shared_ptr<RRTNode> nearest = nullptr;
        for (const auto& n : nodes_) {
            double d = distance(n->x, n->y, x, y);
            if (d < min_dist) { min_dist = d; nearest = n; }
        }
        return nearest;
    }
    
    std::pair<double, double> steer(const std::shared_ptr<RRTNode>& from, 
                                    double to_x, double to_y) const {
        double dx = to_x - from->x, dy = to_y - from->y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < step_size_) return {to_x, to_y};
        return {from->x + dx/dist*step_size_, from->y + dy/dist*step_size_};
    }
    
    double distance(double x1, double y1, double x2, double y2) const {
        return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }
    
    nav_msgs::msg::Path extractPath(const std::shared_ptr<RRTNode>& goal,
                                   const std::string& frame) const {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame;
        path.header.stamp = rclcpp::Clock().now();
        
        std::vector<std::shared_ptr<RRTNode>> waypoints;
        auto curr = goal;
        while (curr) { waypoints.push_back(curr); curr = curr->parent; }
        std::reverse(waypoints.begin(), waypoints.end());
        
        for (const auto& wp : waypoints) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = wp->x;
            pose.pose.position.y = wp->y;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }
};

// ============================================================================
// Simple Path Follower
// ============================================================================
class PathFollower
{
public:
    PathFollower(rclcpp::Node* node) : node_(node), executing_(false), waypoint_idx_(0)
    {
        cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = msg->pose.pose.position.x;
                robot_y_ = msg->pose.pose.position.y;
                auto q = msg->pose.pose.orientation;
                robot_yaw_ = atan2(2.0*(q.w*q.z + q.x*q.y), 
                                  1.0 - 2.0*(q.y*q.y + q.z*q.z));
            });
    }
    
    void followPath(const nav_msgs::msg::Path& path)
    {
        if (path.poses.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Empty path!");
            return;
        }
        
        path_ = path;
        waypoint_idx_ = 0;
        executing_ = true;
        
        RCLCPP_INFO(node_->get_logger(), "Following path with %zu waypoints", path.poses.size());
        
        if (!timer_) {
            timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() { controlLoop(); });
        }
    }
    
    bool isExecuting() const { return executing_; }

private:
    
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path path_;
    bool executing_;
    size_t waypoint_idx_;
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
    
    void controlLoop()
    {
        if (!executing_ || waypoint_idx_ >= path_.poses.size()) {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            if (executing_) {
                RCLCPP_INFO(node_->get_logger(), "✓ Path execution complete!");
                executing_ = false;
            }
            return;
        }
        
        auto& target = path_.poses[waypoint_idx_];
        double dx = target.pose.position.x - robot_x_;
        double dy = target.pose.position.y - robot_y_;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist < 0.2) {
            waypoint_idx_++;
            RCLCPP_INFO(node_->get_logger(), "Waypoint %zu/%zu reached", 
                       waypoint_idx_, path_.poses.size());
            return;
        }
        
        double target_yaw = atan2(dy, dx);
        double angle_diff = target_yaw - robot_yaw_;
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;
        
        geometry_msgs::msg::Twist cmd;
        if (fabs(angle_diff) > 0.3) {
            cmd.angular.z = std::min(1.5, std::max(-1.5, 2.0 * angle_diff));
            cmd.linear.x = 0.05;
        } else {
            cmd.linear.x = std::min(0.22, dist * 0.8);
            cmd.angular.z = std::min(1.0, std::max(-1.0, 1.0 * angle_diff));
        }
        
        cmd_pub_->publish(cmd);
    }
};

// ============================================================================
// ROS 2 Node
// ============================================================================
class RRTPlannerNode : public rclcpp::Node
{
public:
    RRTPlannerNode() : Node("rrt_planner")
    {
        this->declare_parameter("max_iterations", 10000);
        this->declare_parameter("step_size", 0.5);
        this->declare_parameter("goal_bias", 0.1);
        this->declare_parameter("robot_radius", 0.12);
        this->declare_parameter("inflation_radius", 0.15);
        this->declare_parameter("auto_execute", true);
        
        max_iter_ = this->get_parameter("max_iterations").as_int();
        step_size_ = this->get_parameter("step_size").as_double();
        goal_bias_ = this->get_parameter("goal_bias").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        inflation_radius_ = this->get_parameter("inflation_radius").as_double();
        auto_execute_ = this->get_parameter("auto_execute").as_bool();
        
        grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                grid_ = msg;
                planner_ = std::make_unique<RRTPlanner>(
                    *msg, max_iter_, step_size_, goal_bias_, 
                    robot_radius_, inflation_radius_);
                RCLCPP_INFO(this->get_logger(), "Map received: %dx%d", 
                           msg->info.width, msg->info.height);
            });
        
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/start_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                start_ = msg;
                RCLCPP_INFO(this->get_logger(), "Start: (%.2f, %.2f)", 
                           msg->pose.position.x, msg->pose.position.y);
                tryPlan();
            });
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                goal_ = msg;
                RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", 
                           msg->pose.position.x, msg->pose.position.y);
                tryPlan();
            });
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 10);
        tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_tree", 10);
        
        follower_ = std::make_shared<PathFollower>(this);
        
        RCLCPP_INFO(this->get_logger(), "RRT Planner Node Ready!");
        RCLCPP_INFO(this->get_logger(), "  Max iterations: %d", max_iter_);
        RCLCPP_INFO(this->get_logger(), "  Robot radius: %.3f m", robot_radius_);
        RCLCPP_INFO(this->get_logger(), "  Inflation: %.3f m", inflation_radius_);
        RCLCPP_INFO(this->get_logger(), "  Auto execute: %s", auto_execute_ ? "YES" : "NO");
    }

private:
    void tryPlan() {
        if (!grid_ || !start_ || !goal_) return;
        
        std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "=== Planning Path ===");
            auto t1 = std::chrono::high_resolution_clock::now();
            
            try {
                auto path = planner_->plan(*start_, *goal_);
                
                auto t2 = std::chrono::high_resolution_clock::now();
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
                
                RCLCPP_INFO(this->get_logger(), "✓ Path found!");
                RCLCPP_INFO(this->get_logger(), "  Waypoints: %zu", path.poses.size());
                RCLCPP_INFO(this->get_logger(), "  Planning time: %ld ms", ms);
                
                path_pub_->publish(path);
                tree_pub_->publish(planner_->getTreeViz(grid_->header.frame_id));
                
                if (auto_execute_) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    RCLCPP_INFO(this->get_logger(), "=== Executing Path ===");
                    follower_->followPath(path);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "✗ Planning failed: %s", e.what());
            }
        }).detach();
    }
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_, goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
    geometry_msgs::msg::PoseStamped::SharedPtr start_, goal_;
    std::unique_ptr<RRTPlanner> planner_;
    std::shared_ptr<PathFollower> follower_;
    
    int max_iter_;
    double step_size_, goal_bias_, robot_radius_, inflation_radius_;
    bool auto_execute_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTPlannerNode>());
    rclcpp::shutdown();
    return 0;
}