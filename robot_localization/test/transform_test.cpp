#include "robot_localization/utils/transform.hpp"
#include "robot_localization/utils/maths.hpp"
#include "gtest/gtest.h"
using namespace robot_localization::utils;
using namespace robot_localization::mapping;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
TEST(Transform, get_quaternion){
    auto q = quaternionFromYaw(0.4);
    EXPECT_NEAR(q.w, 0.980, 0.01);
    EXPECT_NEAR(q.z, 0.199, 0.01);
}

TEST(Transform, pose_2d){
    Pose2D tranform, source;
    tranform.x = 1.0;
    tranform.y = 2.0;
    tranform.theta = -1.2;
    source.x = 3.0;
    source.y = -12.0;
    source.theta = 3.2;
    auto des = transformPose2D(source, tranform);
    EXPECT_NEAR(des.x, -9.097, 0.01);
    EXPECT_NEAR(des.y, -5.144, 0.01);
    EXPECT_NEAR(des.theta, 2, 0.01);
}

TEST(Transform, point_2d){
    Pose2D tranform; 
    Point32 source;
    tranform.x = 3.0;
    tranform.y = -2.0;
    tranform.theta = 1.5;
    source.x = 3.5;
    source.y = -20.0;
    auto des = transformPoin2D(source, tranform);
    EXPECT_NEAR(des.x, 23.197, 0.01);
    EXPECT_NEAR(des.y, 0.076, 0.01);
}

TEST(Transform, point_array){
    std::vector<float> data = {10.,20.,
                                5.,15.,
                                -7,-10,
                                15.,50.};
    Pose2D tranform; 
    tranform.x = 3.0;
    tranform.y = -5.0;
    tranform.theta = 2.5;
    std::vector<Point32> source;
    for(int i = 0; i < 4; i++){
        Point32 point;
        point.x = data[2*i];
        point.x = data[2*i+1];
        source.push_back(point);
    }
    auto des = transformPointArray(source, tranform);
    EXPECT_NEAR(des[0].x, -13.023, 0.01);
    EXPECT_NEAR(des[0].y, 6.969, 0.01);

    EXPECT_NEAR(des[1].x, -9.017, 0.01);
    EXPECT_NEAR(des[1].y, 3.977, 0.01);

    EXPECT_NEAR(des[2].x, 11.011, 0.01);
    EXPECT_NEAR(des[2].y, -10.985, 0.01);

    EXPECT_NEAR(des[3].x, -37.057, 0.01);
    EXPECT_NEAR(des[3].y, 24.924, 0.01);
}

TEST(Transform, point_cloud){
    LaserScan scan;
    scan.ranges = {10.,1.5,
                    15.,25.,
                    1.2,30,
                    13.,50.};
    scan.angle_min = 1.0;
    scan.angle_increment = 0.1;
    scan.angle_max = scan.angle_min + scan.ranges.size()*scan.angle_increment;
    sensor::PointCloud source(&scan, 20., 2.);
    Pose2D tranform; 
    tranform.x = 2.0;
    tranform.y = -1.0;
    tranform.theta = 2.;
    auto des = transformPointCloud(source, tranform);
    EXPECT_NEAR(des.points[0].x, -7.899, 0.01);
    EXPECT_NEAR(des.points[0].y, 0.411, 0.01);

    EXPECT_NEAR(des.points[1].x, -12.974, 0.01);
    EXPECT_NEAR(des.points[1].y, -1.876, 0.01);

    EXPECT_NEAR(des.points[2].x, -9.658, 0.01);
    EXPECT_NEAR(des.points[2].y, -6.753, 0.01);

    EXPECT_NEAR(des.missing_point[0].x, -17.749, 0.01);
    EXPECT_NEAR(des.missing_point[0].y, -4.155, 0.01);

    EXPECT_NEAR(des.missing_point[1].x, -16.729, 0.01);
    EXPECT_NEAR(des.missing_point[1].y, -8.016, 0.01);

    EXPECT_NEAR(des.missing_point[2].x, -14.962, 0.01);
    EXPECT_NEAR(des.missing_point[2].y, -11.597, 0.01);
}