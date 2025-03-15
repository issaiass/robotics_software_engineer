#include <gtest/gtest.h>
#include "algo/rrt_a_star.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

static nav_msgs::msg::OccupancyGrid createGrid(int width, int height, int val = 0)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = 1.0;  
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    grid.data.resize(width * height, val);
    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Clock().now();
    return grid;
}

// Narrow Corridor
// Occupancy grid with obstacles forming a narrow corridor.
TEST(RRTStarEdgeCasesTest, NarrowCorridor)
{
    auto grid = createGrid(10, 10, 0);

    // narrow corridor
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            if (x != 2) { 
                grid.data[y * 10 + x] = 100; 
            }
        }
    }

    auto path = rrt_a_star(grid);

    EXPECT_TRUE(path.empty()) << "RRT_A* navigated correctly the corridor."; // we expect not to navigate in a 1x1 grid space
}

// Dead End
// Occupancy grid with a dead-end .
TEST(RRTStarEdgeCasesTest, DeadEnd)
{
    auto grid = createGrid(10, 10, 0);

    // blocking wall from (3,0) to (3,9) except at (3,0) or dead end
    for (int y = 1; y < 10; ++y) {
        int index = y * 10 + 3;
        grid.data[index] = 100;
    }

    auto path = rrt_a_star(grid);

    EXPECT_FALSE(path.empty()) << "RRT_A* doesn't found a path in dead-end."; // expect the path to be empty
}

// Open space
// navigation freely
TEST(RRTStarEdgeCasesTest, OpenSpace)
{
    auto grid = createGrid(10, 10, 0);

    auto path = rrt_a_star(grid);


    EXPECT_FALSE(path.empty()) << "RRT_A* failed to find a path in open space.";
}


// Valid path produced with headers.
TEST(RRTAStarTest, ValidPath) {
    auto grid = createGrid(10, 10, 0);

    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Clock().now();

    auto path = rrt_a_star(grid);

    // Validate that the returned path is not empty and that it carries the proper header info.
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front().header.frame_id, grid.header.frame_id);
}

// Obstacles everywhere.
TEST(RRTAStarTest, NoPathFound) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 10;
    grid.info.height = 10;
    grid.info.resolution = 1.0;
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    grid.data.resize(10 * 10, 100);
    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Clock().now();

    auto path = rrt_a_star(grid);
    EXPECT_TRUE(path.empty());     // Expect that no path is found (empty path).
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
