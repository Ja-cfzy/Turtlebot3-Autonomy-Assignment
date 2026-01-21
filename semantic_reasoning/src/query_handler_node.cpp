#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <regex>

// Include dependencies
#include "semantic_database.cpp"
#include "nav_interface.cpp"

// Include generated service header
#include "semantic_reasoning/srv/semantic_query.hpp"

namespace semantic_reasoning {

class QueryHandler : public rclcpp::Node {
public:
    QueryHandler()
        : Node("query_handler") {
        
        // Declare parameters
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("auto_navigate", true);
        this->declare_parameter("json_file_path", "semantic_map.json");
        this->declare_parameter("similarity_threshold", 0.6);
        
        // Get parameters
        map_frame_ = this->get_parameter("map_frame").as_string();
        auto_navigate_ = this->get_parameter("auto_navigate").as_bool();
        json_file_path = this->get_parameter("json_file_path").as_string();
        similarity_threshold_ = this->get_parameter("similarity_threshold").as_double();
        
        // Initialize database
        database_ = std::make_unique<SemanticDatabase>(json_file_path);
        
        // Create service
        service_ = this->create_service<semantic_reasoning::srv::SemanticQuery>(
            "/semantic_query",
            std::bind(&QueryHandler::handleQuery, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create goal publisher for visualization
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/semantic_goal", 10);
        
        RCLCPP_INFO(this->get_logger(), "Query Handler initialized");
    }
    
    void init() {
        // Initialize navigation interface after node construction
        nav_interface_ = std::make_unique<NavInterface>(
            shared_from_this());
    }
    
private:
    
    // Member variables
    std::string map_frame_;
    bool auto_navigate_;
    std::string json_file_path;
    double similarity_threshold_;
    
    rclcpp::Service<semantic_reasoning::srv::SemanticQuery>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    std::unique_ptr<SemanticDatabase> database_;
    std::unique_ptr<NavInterface> nav_interface_;

    void handleQuery(
        const std::shared_ptr<semantic_reasoning::srv::SemanticQuery::Request> request,
        std::shared_ptr<semantic_reasoning::srv::SemanticQuery::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Received query: '%s'", 
                    request->query.c_str());
        
        // Reload database to get latest detections
        database_ = std::make_unique<SemanticDatabase>(json_file_path);
        
        // Extract location from natural language query
        std::string location = extractLocationFromQuery(request->query);
        
        if (location.empty()) {
            RCLCPP_WARN(this->get_logger(), "Could not extract location from query");
            response->found = false;
            return;
        }
        
        // Query database
        auto results = database_->queryByLabel(location, similarity_threshold_);
        
        RCLCPP_DEBUG(this->get_logger(), "Query results: %lu locations found for '%s'", 
                     results.size(), location.c_str());
        
        if (results.empty()) {
            RCLCPP_WARN(this->get_logger(), "No location found for: '%s'", 
                       location.c_str());
            response->found = false;
            return;
        }
        
        // Use the best match (first result)
        const auto& best_match = results[0];
        
        response->found = true;
        response->label = best_match.label;
        response->pose = best_match.pose;
        response->confidence = best_match.confidence;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Found '%s' at (%.2f, %.2f) with confidence %.2f",
                    best_match.label.c_str(),
                    best_match.pose.position.x,
                    best_match.pose.position.y,
                    best_match.confidence);
        
        // Publish goal for visualization
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = map_frame_;
        goal_msg.header.stamp = this->now();
        goal_msg.pose = best_match.pose;
        goal_pub_->publish(goal_msg);
        
        // Optionally navigate to the location
        if (auto_navigate_) {
            RCLCPP_INFO(this->get_logger(), "Initiating navigation to '%s'",
                       best_match.label.c_str());
            nav_interface_->navigateToPose(goal_msg);
        }
    }
    
    std::string extractLocationFromQuery(const std::string& query) {
        // Convert to lowercase for matching
        std::string lower_query = query;
        std::transform(lower_query.begin(), lower_query.end(), 
                      lower_query.begin(), ::tolower);
        
        // Common query patterns
        std::vector<std::regex> patterns = {
            std::regex(R"(where\s+is\s+the\s+(.+))"),
            std::regex(R"(where\s+is\s+(.+))"),
            std::regex(R"(find\s+the\s+(.+))"),
            std::regex(R"(find\s+(.+))"),
            std::regex(R"(go\s+to\s+the\s+(.+))"),
            std::regex(R"(go\s+to\s+(.+))"),
            std::regex(R"(navigate\s+to\s+the\s+(.+))"),
            std::regex(R"(navigate\s+to\s+(.+))"),
            std::regex(R"(take\s+me\s+to\s+the\s+(.+))"),
            std::regex(R"(take\s+me\s+to\s+(.+))"),
            std::regex(R"(location\s+of\s+the\s+(.+))"),
            std::regex(R"(location\s+of\s+(.+))"),
            std::regex(R"(show\s+me\s+the\s+(.+))"),
            std::regex(R"(show\s+me\s+(.+))"),
            std::regex(R"(show\s+the\s+(.+))"),
        };
        
        std::smatch match;
        for (const auto& pattern : patterns) {
            if (std::regex_search(lower_query, match, pattern)) {
                if (match.size() > 1) {
                    std::string location = match[1].str();
                    // Remove leading/trailing whitespace and punctuation
                    const std::string trim_chars = " \t?.!,;:";
                    location.erase(0, location.find_first_not_of(trim_chars));
                    location.erase(location.find_last_not_of(trim_chars) + 1);
                    
                    RCLCPP_INFO(this->get_logger(), "Extracted location: '%s'", location.c_str());
                    return location;
                }
            }
        }
        
        // If no pattern matched, return the whole query as location
        RCLCPP_WARN(this->get_logger(), "No pattern matched, using whole query");
        return lower_query;
    }
};

} // namespace semantic_reasoning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<semantic_reasoning::QueryHandler>();
    node->init();  // Initialize NavInterface after shared_ptr is created
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}