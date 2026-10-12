/******************************************************************************
 * Decoda addition to lslidar_driver.
 *
 * Turns the DIFOP status into a REP 107 diagnostic_msgs/DiagnosticStatus, so
 * any node can read the lidar's health from /diagnostics without depending on
 * lslidar_msgs.
 *****************************************************************************/

#ifndef LSLIDAR_DIFOP_DIAGNOSTICS_HPP
#define LSLIDAR_DIFOP_DIAGNOSTICS_HPP

#include <cstdint>
#include <cstdlib>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "lslidar_driver/difop_status.hpp"

namespace lslidar_driver {

    // Motor setpoints the CH128X1 accepts (5/10/20 Hz). DIFOP reports the
    // measured speed, which sits near but not on a setpoint (e.g. 1201), and
    // the setpoints are 2x apart, so +/-10% bands cannot overlap.
    constexpr uint16_t kMotorSetpointsRpm[] = {300, 600, 1200};
    constexpr double kMotorToleranceRatio = 0.10;

    // True when the measured speed is within tolerance of a valid setpoint.
    // False while spinning up/down between setpoints, or stalled.
    inline bool motorAtSetpoint(uint16_t rpm) {
        for (uint16_t setpoint : kMotorSetpointsRpm) {
            if (std::abs(static_cast<int>(rpm) - setpoint) <= kMotorToleranceRatio * setpoint) {
                return true;
            }
        }
        return false;
    }

    // `name` is the component, e.g. "lslidar: front"; `hardware_id` the serial.
    inline diagnostic_msgs::msg::DiagnosticStatus makeDifopDiagnostic(
            const std::string &name, const std::string &hardware_id, const DifopStatus &status) {
        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name = name;
        diag.hardware_id = hardware_id;
        if (motorAtSetpoint(status.motor_speed_rpm)) {
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diag.message = "OK";
        } else {
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag.message = "motor speed not at a setpoint";
        }

        auto add = [&diag](const char *key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = key;
            kv.value = value;
            diag.values.push_back(kv);
        };
        add("motor_speed_rpm", std::to_string(status.motor_speed_rpm));
        add("time_service_mode", std::to_string(status.time_service_mode));
        // Omitted rather than reported as 255 when the model has no known dust byte.
        if (status.dust_mode != kDustModeUnknown) {
            add("dust_mode", std::to_string(status.dust_mode));
        }
        return diag;
    }

}  // namespace lslidar_driver

#endif  // LSLIDAR_DIFOP_DIAGNOSTICS_HPP
