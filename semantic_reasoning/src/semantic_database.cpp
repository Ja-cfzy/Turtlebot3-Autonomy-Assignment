#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>

namespace semantic_reasoning {

// Unified Location structure for both runtime and database use
struct Location {
    int id = -1;                               // -1 for runtime, >=0 for database
    std::string label;
    geometry_msgs::msg::Pose pose;             // Position + Orientation for navigation
    double confidence = 0.0;
    std::vector<std::string> synonyms;
    int detection_count = 0;
    std::vector<std::string> image_paths;     // All images in detection history
    std::vector<std::string> timestamps;      // All timestamps in detection history
    
    nlohmann::ordered_json toJson() const {
        nlohmann::ordered_json j;
        j["id"] = id;
        j["label"] = label;
        j["synonyms"] = synonyms;
        j["position"]["x"] = pose.position.x;
        j["position"]["y"] = pose.position.y;
        j["position"]["z"] = pose.position.z;
        j["orientation"]["x"] = pose.orientation.x;
        j["orientation"]["y"] = pose.orientation.y;
        j["orientation"]["z"] = pose.orientation.z;
        j["orientation"]["w"] = pose.orientation.w;
        j["confidence"] = confidence;
        j["detection_count"] = detection_count;
        j["timestamps"] = timestamps;           // Store ALL timestamps as array
        j["image_paths"] = image_paths;         // Store ALL image paths as array
        return j;
    }
    
    static Location fromJson(const nlohmann::json& j) {
        Location loc;
        loc.id = j["id"];
        loc.label = j["label"];
        loc.synonyms = j["synonyms"].get<std::vector<std::string>>();
        loc.pose.position.x = j["position"]["x"];
        loc.pose.position.y = j["position"]["y"];
        loc.pose.position.z = j["position"]["z"];
        loc.pose.orientation.x = j["orientation"]["x"];
        loc.pose.orientation.y = j["orientation"]["y"];
        loc.pose.orientation.z = j["orientation"]["z"];
        loc.pose.orientation.w = j["orientation"]["w"];
        loc.confidence = j["confidence"];
        loc.detection_count = j["detection_count"];
        // Load full detection history from arrays
        if (!j["timestamps"].is_null()) {
            loc.timestamps = j["timestamps"].get<std::vector<std::string>>();
        }
        if (!j["image_paths"].is_null()) {
            loc.image_paths = j["image_paths"].get<std::vector<std::string>>();
        }
        return loc;
    }
};

// SemanticDatabase class
class SemanticDatabase {
public:
    SemanticDatabase(const std::string& db_path = "semantic_map.json")
        : db_path_(db_path), next_id_(0) {
        load();
    }
    
    ~SemanticDatabase() {
    }
    
    bool addLocation(const Location& location) {
        Location new_loc = location;
        new_loc.id = next_id_++;
        locations_.push_back(new_loc);
        return save();
    }
    
    std::vector<Location> queryByLabel(const std::string& label, 
                                               double threshold = 0.7) {
        std::vector<Location> results;
        
        for (const auto& loc : locations_) {
            double sim = stringSimilarity(label, loc.label);
            if (sim >= threshold) {
                results.push_back(loc);
                continue;
            }
            
            for (const auto& synonym : loc.synonyms) {
                sim = stringSimilarity(label, synonym);
                if (sim >= threshold) {
                    results.push_back(loc);
                    break;
                }
            }
        }
        
        return results;
    }
    
    std::vector<Location> queryByPosition(double x, double y, 
                                                  double radius = 1.0) {
        std::vector<Location> results;
        
        for (const auto& loc : locations_) {
            double dx = loc.pose.position.x - x;
            double dy = loc.pose.position.y - y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist <= radius) {
                results.push_back(loc);
            }
        }
        
        return results;
    }
    
    bool save() {
        nlohmann::ordered_json j;
        j["locations"] = nlohmann::ordered_json::array();
        
        for (const auto& loc : locations_) {
            j["locations"].push_back(loc.toJson());
        }
        
        std::ofstream file(db_path_);
        if (!file.is_open()) {
            return false;
        }
        
        file << j.dump(2);
        file.close();
        return true;
    }
    
    bool load() {
        std::ifstream file(db_path_);
        if (!file.is_open()) {
            return false;
        }
        
        nlohmann::ordered_json j;
        file >> j;
        file.close();
        
        locations_.clear();
        for (const auto& item : j["locations"]) {
            locations_.push_back(Location::fromJson(item));
            next_id_ = std::max(next_id_, locations_.back().id + 1);
        }
        
        return true;
    }
    
    void clear() {
        locations_.clear();
        next_id_ = 0;
    }
    
private:
    std::string db_path_;
    std::vector<Location> locations_;
    int next_id_;
    
    double stringSimilarity(const std::string& s1, const std::string& s2) const {
        std::string str1 = s1;
        std::string str2 = s2;
        
        std::transform(str1.begin(), str1.end(), str1.begin(), ::tolower);
        std::transform(str2.begin(), str2.end(), str2.begin(), ::tolower);
        
        const size_t len1 = str1.size(), len2 = str2.size();
        std::vector<std::vector<unsigned int>> d(len1 + 1, 
                                                 std::vector<unsigned int>(len2 + 1));
        
        d[0][0] = 0;
        for(unsigned int i = 1; i <= len1; ++i) d[i][0] = i;
        for(unsigned int i = 1; i <= len2; ++i) d[0][i] = i;
        
        for(unsigned int i = 1; i <= len1; ++i) {
            for(unsigned int j = 1; j <= len2; ++j) {
                d[i][j] = std::min({
                    d[i-1][j] + 1,
                    d[i][j-1] + 1,
                    d[i-1][j-1] + (str1[i-1] == str2[j-1] ? 0 : 1)
                });
            }
        }
        
        unsigned int max_len = std::max(len1, len2);
        return max_len > 0 ? 1.0 - (double)d[len1][len2] / max_len : 1.0;
    }
};

} // namespace semantic_reasoning