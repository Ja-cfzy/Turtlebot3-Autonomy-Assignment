#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <iomanip>
#include <sstream>

// Include the database and VLM implementations
#include "semantic_database.cpp"
#include "mock_vlm.cpp"

namespace semantic_reasoning {

class SemanticMapper : public rclcpp::Node {
public:
    SemanticMapper()
        : Node("semantic_mapper"),
          first_capture_(true),
          image_counter_(0), 
          latest_image_(nullptr){
        
        // Declare parameters
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_frame", "base_footprint");
        this->declare_parameter("capture_interval", 2.0);
        this->declare_parameter("min_distance_between_captures", 0.5);
        this->declare_parameter("camera_topic", "/camera/image_raw");
        this->declare_parameter("camera_offset_x", 0.5);  
        this->declare_parameter("deduplication_distance", 1.0);  
        this->declare_parameter("save_images", true);
        this->declare_parameter("save_images_path", "captures/");
        this->declare_parameter("json_file_path", "semantic_map.json");

        // Get parameters
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        capture_interval_ = this->get_parameter("capture_interval").as_double();
        min_distance_between_captures_ = this->get_parameter("min_distance_between_captures").as_double();
        camera_topic = this->get_parameter("camera_topic").as_string();
        camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
        dedup_distance_ = this->get_parameter("deduplication_distance").as_double();
        save_images_ = this->get_parameter("save_images").as_bool();
        save_images_path_ = this->get_parameter("save_images_path").as_string();
        json_file_path_ = this->get_parameter("json_file_path").as_string();

        
        // Create captures directory if saving images
        if (save_images_) {
            std::string mkdir_cmd = "mkdir -p " + save_images_path_;
            std::system(mkdir_cmd.c_str());
            RCLCPP_INFO(this->get_logger(), "Image saving enabled - directory created: %s", save_images_path_.c_str());
        }
        
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize semantic components
        database_ = std::make_unique<SemanticDatabase>(json_file_path_);
        vlm_ = std::make_unique<MockVLM>(dedup_distance_);
        
        // Create subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&SemanticMapper::imageCallback, this, std::placeholders::_1));
        
        // Create publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/semantic_markers", 10);
        
        // Create timer for periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(capture_interval_),
            std::bind(&SemanticMapper::timerCallback, this));
        
        // Create timer for periodic database saves
        database_save_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(5.0),
            std::bind(&SemanticMapper::databaseSaveCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Semantic Mapper initialized");
    }
    
    ~SemanticMapper() {
        RCLCPP_INFO(this->get_logger(), "Saving semantic map...");
        saveAllDetectionsToDatabase();
    }
    
private:
    
    // Member variables
    std::string map_frame_;
    std::string base_frame_;
    double capture_interval_;
    double min_distance_between_captures_;
    std::string camera_topic;
    double camera_offset_x_;  
    double dedup_distance_;     // Member variable for deduplication distance
    bool save_images_;
    std::string save_images_path_;
    std::string json_file_path_;     // Member variable for JSON file path
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr database_save_timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::unique_ptr<SemanticDatabase> database_;  // Persistent storage
    std::unique_ptr<MockVLM> vlm_;                  // Real-time deduplication
    
    geometry_msgs::msg::Pose last_capture_pose_;
    bool first_capture_;
    int image_counter_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Images are stored for processing in timer callback
        latest_image_ = msg;
    }
    
    void timerCallback() {
        geometry_msgs::msg::Pose current_pose;
        
        if (!getRobotPose(current_pose)) {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose");
            return;
        }
        
        // Check if we should capture at this location
        if (!first_capture_ && !isNewLocation(current_pose.position.x, 
                                             current_pose.position.y)) {
            return;
        }
        
        // Process image and semantic labeling
        cv::Mat image;
        if (latest_image_) {
            try {
                auto cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
                image = cv_ptr->image;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                image = cv::Mat();  // Use empty image
            }
        }
        
        // Process image and semantic labeling
        processImage(image, current_pose);
        
        last_capture_pose_ = current_pose;
        first_capture_ = false;
        
        // Publish visualization
        publishVisualization();
    }
    
    bool getRobotPose(geometry_msgs::msg::Pose& pose) {
        try {
            auto transform = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
            
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            pose.orientation = transform.transform.rotation;
            
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return false;
        }
    }
    
    void processImage(const cv::Mat& image, const geometry_msgs::msg::Pose& pose) {
        // Calculate offset position in front of robot (in camera view)
        geometry_msgs::msg::Pose offset_pose = calculateOffsetPose(pose, camera_offset_x_);

        // Use VLM to classify location
        VLMResult vlm_result = vlm_->classifyLocation(image, offset_pose);
        
        if (vlm_result.label == "unknown") {
            RCLCPP_DEBUG(this->get_logger(), "No semantic label detected");
            return;
        }

        // Generate timestamp using ROS2 simulation time
        auto now = this->now();
        std::string timestamp = std::to_string(now.seconds()) + "." + 
                                std::to_string(now.nanoseconds() % 1000000000);
        
        // Save image if enabled and image is valid
        std::string image_filename = "img_" + std::to_string(image_counter_++) + ".jpg";
        std::string image_path = save_images_path_ + image_filename;
        
        if (save_images_ && !image.empty()) {
            if (cv::imwrite(image_path, image)) {
                RCLCPP_INFO(this->get_logger(), "Saved image: %s", image_path.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to save image: %s", image_path.c_str());
            }
        }
        
        // Add to VLM's detected locations (with deduplication and image info)
        vlm_->addDetectedLocation(
            vlm_result.label,
            offset_pose,
            vlm_result.confidence,
            vlm_result.synonyms,
            image_path,
            timestamp
        );
    }

    geometry_msgs::msg::Pose calculateOffsetPose(const geometry_msgs::msg::Pose& robot_pose, 
                                                  double offset_distance) {
        // Calculate position offset_distance meters in front of robot
        // Robot's orientation is given by quaternion
        geometry_msgs::msg::Pose offset_pose = robot_pose;
        
        // Extract yaw from quaternion
        double siny_cosp = 2.0 * (robot_pose.orientation.w * robot_pose.orientation.z + 
                                  robot_pose.orientation.x * robot_pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (robot_pose.orientation.y * robot_pose.orientation.y + 
                                       robot_pose.orientation.z * robot_pose.orientation.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // Calculate offset in global frame
        offset_pose.position.x = robot_pose.position.x + offset_distance * std::cos(yaw);
        offset_pose.position.y = robot_pose.position.y + offset_distance * std::sin(yaw);
        
        return offset_pose;
    }
    
    void databaseSaveCallback() {
        // Periodically save deduplicated detections to database
        auto detected_locs = vlm_->getDetectedLocations();
        if (!detected_locs.empty()) {
            // Clear previous data and save current deduplicated state
            database_->clear();
            saveAllDetectionsToDatabase();
        }
    }
    
    void publishVisualization() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        // Visualize real-time deduplicated locations from VLM
        auto detected_locs = vlm_->getDetectedLocations();
        
        int id = 0;
        for (const auto& det_loc : detected_locs) {
            // Text marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = this->now();
            marker.ns = "semantic_labels";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = det_loc.pose.position.x;
            marker.pose.position.y = det_loc.pose.position.y;
            marker.pose.position.z = 0.5;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.z = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            marker.text = det_loc.label + " (x" + std::to_string(det_loc.detection_count) + ")";
            marker_array.markers.push_back(marker);
            
            // Sphere marker
            visualization_msgs::msg::Marker sphere;
            sphere.header = marker.header;
            sphere.ns = "semantic_locations";
            sphere.id = id++;
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = det_loc.pose.position.x;
            sphere.pose.position.y = det_loc.pose.position.y;
            sphere.pose.position.z = 0.0;
            sphere.pose.orientation.w = 1.0;
            
            sphere.scale.x = 0.3;
            sphere.scale.y = 0.3;
            sphere.scale.z = 0.3;
            
            sphere.color.r = 1.0;  // Red - high confidence
            sphere.color.g = 0.0;
            sphere.color.b = 1.0;
            sphere.color.a = 0.7;
            
            marker_array.markers.push_back(sphere);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    bool isNewLocation(double x, double y) {
        double dx = x - last_capture_pose_.position.x;
        double dy = y - last_capture_pose_.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        return dist >= min_distance_between_captures_;
    }
    
    void saveAllDetectionsToDatabase() {
        // Save deduplicated locations from VLM to persistent database
        auto detected_locs = vlm_->getDetectedLocations();
        
        for (const auto& det_loc : detected_locs) {
            Location db_loc = det_loc;  // Copy all fields including pose
            database_->addLocation(db_loc);
        }
    }
};

} // namespace semantic_reasoning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<semantic_reasoning::SemanticMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}