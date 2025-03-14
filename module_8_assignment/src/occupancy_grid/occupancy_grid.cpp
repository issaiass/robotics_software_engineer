#include "occupancy_grid/occupancy_grid.hpp"

OccupancyGrid_Publisher::OccupancyGrid_Publisher() : Node("occupancy_grid_publisher") {
    RCLCPP_INFO(this->get_logger(), "Publishing Occupancy Grid");

    og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
    og_timer = this->create_wall_timer(500ms, std::bind(&OccupancyGrid_Publisher::og_callback, this));
}

void OccupancyGrid_Publisher::og_callback() {
    auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
    int grid_width = 10, grid_height = 10;
    int area = grid_height * grid_width;
    std::vector<signed char> og_array(area, 0);   // Initialize all cells as free

    add_line = [&](int row) {
        for (int x = 2; x < grid_width - 2; ++x) {   // Start from 2 and end 2 cells before the edge
          og_array[row * grid_width + x] = 100;     // Set cells to occupied
        }
      };


    add_diag = [&](int row) {
        for (int x = 1; x < grid_width - 1; ++x) {   
          og_array[(row + x) * grid_width + x] = 100;
        }
      };

    // Adding lines on the 2nd, 6th, and 9th rows
    if (grid_height > 3) add_diag(3);
    if (grid_height > 1) add_line(1);
    if (grid_height > 5) add_line(5);

    // og_array[99]=100; // finding starting point
    occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
    occupancy_grid_msg.header.frame_id = "map_frame";

    occupancy_grid_msg.info.resolution = 1;

    occupancy_grid_msg.info.width = grid_width;
    occupancy_grid_msg.info.height = grid_height;

    occupancy_grid_msg.info.origin.position.x = -5.0;
    occupancy_grid_msg.info.origin.position.y = -5.0;
    occupancy_grid_msg.info.origin.position.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;
    occupancy_grid_msg.data = og_array;

    og_pub->publish(occupancy_grid_msg);
}