#ifndef LOCALIZATION_STACK_RIGID_TRANSFORM__HPP__
#define LOCALIZATION_STACK_RIGID_TRANSFORM__HPP__

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/pose.hpp"
#include "absl/strings/substitute.h"

namespace localization_node{
namespace transform{

template <typename FloatType>
class Rigid3 {
    public:
        using Vector = Eigen::Matrix<FloatType, 3, 1>;
        using Quaternion = Eigen::Quaternion<FloatType>;
        using AngleAxis = Eigen::AngleAxis<FloatType>;

        Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}

        Rigid3(const Vector& translation, const Quaternion& rotation)
            : translation_(translation), rotation_(rotation) {}

        Rigid3(const Vector& translation, const AngleAxis& rotation)
            : translation_(translation), rotation_(rotation) {}

        Rigid3(const geometry_msgs::msg::Pose & pose)
            : translation_({pose.position.x, pose.position.y, pose.position.z}),
            rotation_({pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w}) {}

        static Rigid3 rotation(const AngleAxis& angle_axis) {
            return Rigid3(Vector::Zero(), Quaternion(angle_axis));
        }

        static Rigid3 rotation(const Quaternion& rotation) {
            return Rigid3(Vector::Zero(), rotation);
        }

        static Rigid3 translation(const Vector& vector) {
            return Rigid3(vector, Quaternion::Identity());
        }

        static Rigid3 fromArrays(const std::array<FloatType, 4>& rotation,
                                const std::array<FloatType, 3>& translation) {
            return Rigid3(Eigen::Map<const Vector>(translation.data()),
                        Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                                    rotation[2], rotation[3]));
        }

        static Rigid3<FloatType> identity() { return Rigid3<FloatType>(); }

        template <typename OtherType>
        Rigid3<OtherType> cast() const {
            return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                                    rotation_.template cast<OtherType>());
        }

        const Vector& translation() const { return translation_; }

        const Quaternion& rotation() const { return rotation_; }

        Rigid3 inverse() const {
            const Quaternion rotation = rotation_.conjugate();
            const Vector translation = -(rotation * translation_);
            return Rigid3(translation, rotation);
        }

        Rigid3 operator*(const Rigid3& other) const {
            // Transform translation
            Vector new_translation = rotation_ * other.translation_ + translation_;
            // Transform rotation
            Quaternion new_rotation = rotation_ * other.rotation_;

            return Rigid3(new_translation, new_rotation);
        }

        std::string debugString() const {
            return absl::Substitute("{ t: [$0, $1, $2], q: [$3, $4, $5, $6] }",
                                    translation().x(), translation().y(),
                                    translation().z(), rotation().w(), rotation().x(),
                                    rotation().y(), rotation().z());
        }

        bool isValid() const {
            return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
                !std::isnan(translation_.z()) &&
                std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
        }

    private:
        Vector translation_;
        Quaternion rotation_;
};

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;
}
}

#endif