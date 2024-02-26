#ifndef LOCALIZATION_STACK_ODOM_INTERFACE__HPP__
#define LOCALIZATION_STACK_ODOM_INTERFACE__HPP__

#include "rclcpp/rclcpp.hpp"
#include "localization_node/transform/rigid_transform.hpp"
#include "memory.h"
#include "tf2_ros/buffer.h"
#include <array>

namespace localization_node{
namespace odom_provider{

enum OdomSourceType { RosTF, IMU, ENCODER, UNKNOW };

class OdomInterface
{
public:
    virtual ~OdomInterface();

    virtual void setUp(std::unique_ptr<rclcpp::Node> parent, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

    virtual void updateRobotPose(const transform::Rigid3f& robot_pose) = 0;

    virtual transform::Rigid3f getCurrentRobotPose() = 0;

    virtual std::array<float,2> getPoseDiff(const transform::Rigid3f& pose_check) = 0;

    virtual std::vector<OdomSourceType> getOdomSourceType();

};

}
}


#endif