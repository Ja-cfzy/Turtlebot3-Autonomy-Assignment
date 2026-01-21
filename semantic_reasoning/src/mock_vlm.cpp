#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace semantic_reasoning {

struct VLMResult {
    std::string label;
    double confidence;
    std::vector<std::string> synonyms;
};

struct MockLocation {
    double x, y;
    std::string label;
    std::vector<std::string> synonyms;
    double detection_radius;
};

// ============================================================================
// MockVLM - Position-based with Deduplication
// ============================================================================
class MockVLM {
public:
    MockVLM(double dedup_distance = 1.0) 
        : dedup_distance_(dedup_distance) {
        
        // Initialize with predefined mock locations
        // Adjust these coordinates to match your Gazebo world layout
        addMockLocation(6.5, 3.5, "meeting room", {"conference room", "meeting area"});
        addMockLocation(6.5, -1.5, "meeting room", {"conference room", "meeting area"});
        addMockLocation(-4.0, -2.0, "pantry", {"kitchen", "break room"});
        addMockLocation(3.0, 1.0, "entrance", {"lobby", "reception"});
        addMockLocation(-8.0, 1.0, "office", {"workspace", "desk area", "workstation"});
        addMockLocation(-6.0, -6.0, "office", {"workspace", "desk area", "workstation"});
        addMockLocation(13.0, 7.0, "office", {"workspace", "desk area", "workstation"});
        addMockLocation(20.0, -1.0, "store room", {"storage room", "stock room"});
    }
    
    VLMResult classifyLocation(const cv::Mat& image, 
                               const geometry_msgs::msg::Pose& pose) {
        // Suppress unused parameter warning (image not used in mock mode)
        (void)image;
        
        VLMResult result;
        result.label = "unknown";
        result.confidence = 0.0;
        
        double robot_x = pose.position.x;
        double robot_y = pose.position.y;
        
        // Find nearest mock location
        double min_dist = std::numeric_limits<double>::max();
        const MockLocation* nearest = nullptr;
        
        for (const auto& loc : mock_locations_) {
            double dist = distance(robot_x, robot_y, loc.x, loc.y);
            if (dist < min_dist && dist < loc.detection_radius) {
                min_dist = dist;
                nearest = &loc;
            }
        }
        
        if (nearest != nullptr) {
            result.label = nearest->label;
            result.synonyms = nearest->synonyms;
            // Confidence decreases with distance
            result.confidence = 1.0 - (min_dist / nearest->detection_radius);
            result.confidence = std::max(0.5, result.confidence);
        }
        
        return result;
    }
    
    bool addDetectedLocation(const std::string& label, const geometry_msgs::msg::Pose& pose,
                            double confidence, const std::vector<std::string>& synonyms,
                            const std::string& image_path = "", const std::string& timestamp = "") {
        // Check if a similar location already exists nearby
        for (auto& existing : detected_locations_) {
            double dist = distance(pose.position.x, pose.position.y, existing.pose.position.x, existing.pose.position.y);
            
            // If location is nearby and has same label, merge them
            if (dist < dedup_distance_ && existing.label == label) {
                // Update with weighted average position based on confidence
                double total_conf = existing.confidence + confidence;
                existing.pose.position.x = (existing.pose.position.x * existing.confidence + pose.position.x * confidence) / total_conf;
                existing.pose.position.y = (existing.pose.position.y * existing.confidence + pose.position.y * confidence) / total_conf;
                existing.pose.position.z = (existing.pose.position.z * existing.confidence + pose.position.z * confidence) / total_conf;
                
                // Keep higher confidence
                existing.confidence = std::max(existing.confidence, confidence);
                existing.detection_count++;
                
                // Append detection history (image and timestamp)
                if (!image_path.empty()) {
                    existing.image_paths.push_back(image_path);
                }
                if (!timestamp.empty()) {
                    existing.timestamps.push_back(timestamp);
                }
                
                // Merge synonyms (avoid duplicates)
                for (const auto& syn : synonyms) {
                    if (std::find(existing.synonyms.begin(), 
                                  existing.synonyms.end(), syn) == existing.synonyms.end()) {
                        existing.synonyms.push_back(syn);
                    }
                }
                
                return false; // Location was merged, not added
            }
        }
        
        // No similar location found, add as new with actual robot pose
        Location new_loc;
        new_loc.label = label;
        new_loc.pose = pose;  // Store actual robot pose with real orientation
        new_loc.confidence = confidence;
        new_loc.synonyms = synonyms;
        new_loc.detection_count = 1;
        if (!image_path.empty()) {
            new_loc.image_paths.push_back(image_path);
        }
        if (!timestamp.empty()) {
            new_loc.timestamps.push_back(timestamp);
        }
        
        detected_locations_.push_back(new_loc);
        return true; // New location was added
    }
    
    // Get all detected locations
    const std::vector<Location>& getDetectedLocations() const {
        return detected_locations_;
    }
    
    void addMockLocation(double x, double y, const std::string& label,
                         const std::vector<std::string>& synonyms = {}) {
        MockLocation loc;
        loc.x = x;
        loc.y = y;
        loc.label = label;
        loc.synonyms = synonyms;
        loc.detection_radius = 1.5; // 1.5 meters
        
        mock_locations_.push_back(loc);
    }
    
private:
    std::vector<MockLocation> mock_locations_;
    std::vector<Location> detected_locations_;
    double dedup_distance_;
    
    double distance(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx*dx + dy*dy);
    }
};

} // namespace semantic_reasoning