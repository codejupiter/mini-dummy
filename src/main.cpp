#include <cctype>
#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"

#include "kinematics.h"

namespace {

constexpr std::size_t kLineCapacity = 96;

mini_dummy::Kinematics kinematics{};
mini_dummy::JointAngles current_target{};
bool has_target = false;

bool equals_ignore_case(const char* left, const char* right) {
    while (*left != '\0' && *right != '\0') {
        const auto left_char = static_cast<unsigned char>(*left);
        const auto right_char = static_cast<unsigned char>(*right);
        if (std::toupper(left_char) != std::toupper(right_char)) {
            return false;
        }
        ++left;
        ++right;
    }
    return *left == '\0' && *right == '\0';
}

bool parse_g0(const char* line, float& x_mm, float& y_mm, float& z_mm) {
    char command[8]{};
    char extra[2]{};
    const int fields = std::sscanf(
        line,
        "%7s %f %f %f %1s",
        command,
        &x_mm,
        &y_mm,
        &z_mm,
        extra);
    return fields == 4 && equals_ignore_case(command, "G0");
}

void apply_joint_targets(const mini_dummy::JointAngles& joints) {
    // Hardware-specific servo output belongs here. Convert mathematical joint
    // angles to calibrated servo pulse widths only after setting mechanical
    // zero offsets, direction signs, and safe per-joint limits.
    (void)joints;
}

void print_target(const mini_dummy::JointAngles& target) {
    std::printf(
        "Q0=%.2f Q1=%.2f Q2=%.2f Q3=%.2f",
        static_cast<double>(target.base_deg),
        static_cast<double>(target.shoulder_deg),
        static_cast<double>(target.elbow_deg),
        static_cast<double>(target.wrist_deg));
}

void process_command(const char* line) {
    if (equals_ignore_case(line, "PING")) {
        std::printf("OK PONG\r\n");
        return;
    }

    if (equals_ignore_case(line, "HELP")) {
        std::printf("OK COMMANDS: G0 <X_MM> <Y_MM> <Z_MM> | M114 | PING\r\n");
        return;
    }

    if (equals_ignore_case(line, "M114")) {
        if (!has_target) {
            std::printf("ERR NO_TARGET\r\n");
            return;
        }
        std::printf("OK ");
        print_target(current_target);
        std::printf("\r\n");
        return;
    }

    float x_mm = 0.0F;
    float y_mm = 0.0F;
    float z_mm = 0.0F;
    if (!parse_g0(line, x_mm, y_mm, z_mm)) {
        std::printf("ERR BAD_COMMAND USE: G0 <X_MM> <Y_MM> <Z_MM>\r\n");
        return;
    }

    const mini_dummy::JointAngles target = kinematics.solve(x_mm, y_mm, z_mm);
    if (!target.reachable) {
        std::printf("ERR UNREACHABLE X=%.2f Y=%.2f Z=%.2f\r\n",
            static_cast<double>(x_mm),
            static_cast<double>(y_mm),
            static_cast<double>(z_mm));
        return;
    }

    current_target = target;
    has_target = true;
    apply_joint_targets(current_target);

    std::printf("OK G0 X=%.2f Y=%.2f Z=%.2f ",
        static_cast<double>(x_mm),
        static_cast<double>(y_mm),
        static_cast<double>(z_mm));
    print_target(current_target);
    std::printf("\r\n");
}

}  // namespace

int main() {
    stdio_init_all();

    // Give the host a moment to enumerate USB-CDC after reset.
    sleep_ms(1200);
    std::printf("READY MINI-DUMMY 1.0\r\n");
    std::printf("TYPE HELP FOR COMMANDS\r\n");

    char line[kLineCapacity]{};
    std::size_t length = 0;
    bool overflowed = false;

    while (true) {
        const int character = getchar_timeout_us(1000);
        if (character == PICO_ERROR_TIMEOUT) {
            tight_loop_contents();
            continue;
        }

        if (character == '\r' || character == '\n') {
            if (overflowed) {
                std::printf("ERR LINE_TOO_LONG\r\n");
            } else if (length > 0) {
                line[length] = '\0';
                process_command(line);
            }
            length = 0;
            overflowed = false;
            continue;
        }

        if (overflowed) {
            continue;
        }

        if (length + 1 >= kLineCapacity) {
            overflowed = true;
            continue;
        }

        line[length++] = static_cast<char>(character);
    }
}
