/******************************************************************************
 * Decoda addition to lslidar_driver.
 *
 * Runtime status fields of a CH-series DIFOP packet that the vendor driver
 * reads (or never reads) but does not publish. Published on
 * lslidar_device_info alongside the vendor fields.
 *
 * Byte 51 (dust mode) is empirically determined on the CH128X1 and is not in
 * the vendor manual -- see provision_lslidar.py in ccm-provisioning.
 *****************************************************************************/

#ifndef LSLIDAR_DIFOP_STATUS_HPP
#define LSLIDAR_DIFOP_STATUS_HPP

#include <cstddef>
#include <cstdint>

namespace lslidar_driver {

    constexpr std::size_t kDifopMotorSpeedOffset = 8;
    constexpr std::size_t kDifopTimeServiceOffset = 44;
    constexpr std::size_t kDifopDustOffset = 51;
    constexpr uint8_t kDustModeUnknown = 255;

    struct DifopStatus {
        uint16_t motor_speed_rpm = 0;
        uint8_t time_service_mode = 0;
        uint8_t dust_mode = kDustModeUnknown;
    };

    // `data` must hold at least 52 bytes (callers pass a validated 1206-byte DIFOP).
    inline DifopStatus parseChDifopStatus(const uint8_t *data, bool is_ch128x1) {
        DifopStatus status;
        status.motor_speed_rpm = static_cast<uint16_t>(
            (data[kDifopMotorSpeedOffset] << 8) | data[kDifopMotorSpeedOffset + 1]);
        status.time_service_mode = data[kDifopTimeServiceOffset];
        if (is_ch128x1) status.dust_mode = data[kDifopDustOffset];
        return status;
    }

}  // namespace lslidar_driver

#endif  // LSLIDAR_DIFOP_STATUS_HPP
