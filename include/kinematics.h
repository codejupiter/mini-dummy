#pragma once

#include <cmath>

namespace mini_dummy {

constexpr float kPi = 3.14159265358979323846F;

constexpr float degrees_to_radians(float degrees) {
    return degrees * (kPi / 180.0F);
}

constexpr float radians_to_degrees(float radians) {
    return radians * (180.0F / kPi);
}

struct ArmGeometry {
    float base_height_mm{60.0F};
    float upper_arm_mm{110.0F};
    float forearm_mm{105.0F};
    float tool_offset_mm{45.0F};
    float tool_pitch_deg{-20.0F};
};

struct JointRange {
    float minimum_deg;
    float maximum_deg;
};

struct JointLimits {
    JointRange base{-180.0F, 180.0F};
    JointRange shoulder{-20.0F, 150.0F};
    JointRange elbow{-150.0F, 150.0F};
    JointRange wrist{-150.0F, 150.0F};
};

struct JointAngles {
    float base_deg{0.0F};
    float shoulder_deg{0.0F};
    float elbow_deg{0.0F};
    float wrist_deg{0.0F};
    bool reachable{false};
};

class Kinematics {
public:
    explicit constexpr Kinematics(
        ArmGeometry geometry = {},
        JointLimits limits = {})
        : geometry_(geometry), limits_(limits) {}

    [[nodiscard]] JointAngles solve(
        float x_mm,
        float y_mm,
        float z_mm,
        bool elbow_up = false) const {
        JointAngles result{};

        const float base_rad = std::atan2(y_mm, x_mm);
        const float radial_mm = std::hypot(x_mm, y_mm);
        const float tool_pitch_rad = degrees_to_radians(geometry_.tool_pitch_deg);

        // Remove the final tool link so the shoulder/elbow pair solves for the
        // wrist center rather than the tool tip.
        const float wrist_r_mm =
            radial_mm - geometry_.tool_offset_mm * std::cos(tool_pitch_rad);
        const float wrist_z_mm =
            z_mm - geometry_.base_height_mm
            - geometry_.tool_offset_mm * std::sin(tool_pitch_rad);

        const float upper = geometry_.upper_arm_mm;
        const float fore = geometry_.forearm_mm;
        const float numerator =
            wrist_r_mm * wrist_r_mm + wrist_z_mm * wrist_z_mm
            - upper * upper - fore * fore;
        const float denominator = 2.0F * upper * fore;

        if (denominator <= 0.0F) {
            return result;
        }

        float cosine_elbow = numerator / denominator;
        constexpr float kTolerance = 1.0e-5F;
        if (cosine_elbow < -1.0F - kTolerance
            || cosine_elbow > 1.0F + kTolerance) {
            return result;
        }

        cosine_elbow = clamp(cosine_elbow, -1.0F, 1.0F);
        float elbow_rad = std::acos(cosine_elbow);
        if (elbow_up) {
            elbow_rad = -elbow_rad;
        }

        const float shoulder_rad =
            std::atan2(wrist_z_mm, wrist_r_mm)
            - std::atan2(
                fore * std::sin(elbow_rad),
                upper + fore * std::cos(elbow_rad));
        const float wrist_rad = tool_pitch_rad - shoulder_rad - elbow_rad;

        result.base_deg = radians_to_degrees(base_rad);
        result.shoulder_deg = radians_to_degrees(shoulder_rad);
        result.elbow_deg = radians_to_degrees(elbow_rad);
        result.wrist_deg = radians_to_degrees(wrist_rad);
        result.reachable = within_limits(result);
        return result;
    }

    [[nodiscard]] constexpr const ArmGeometry& geometry() const {
        return geometry_;
    }

private:
    static constexpr float clamp(float value, float low, float high) {
        return value < low ? low : (value > high ? high : value);
    }

    static constexpr bool within(float value, JointRange range) {
        return value >= range.minimum_deg && value <= range.maximum_deg;
    }

    [[nodiscard]] constexpr bool within_limits(const JointAngles& joints) const {
        return within(joints.base_deg, limits_.base)
            && within(joints.shoulder_deg, limits_.shoulder)
            && within(joints.elbow_deg, limits_.elbow)
            && within(joints.wrist_deg, limits_.wrist);
    }

    ArmGeometry geometry_;
    JointLimits limits_;
};

}  // namespace mini_dummy
