/******************************************************************************
 * Decoda addition to lslidar_driver.
 *
 * Publishes the CH driver's cloud directly in detect-dev's UnifiedPoint layout
 * and caps its output rate, so an LSLidar truck needs no sensor_adaptor hop.
 *
 * The layout MUST stay byte-identical to detect::common::UnifiedPoint in
 * detect-dev src/common/include/common/point_types.h, and the field list to
 * makeUnifiedCloud() in src/common/src/sensor_adaptor/sensor_adaptor.cpp.
 * dust_filter and lidar_merger read these bytes without checking field names,
 * so a drift between the two copies corrupts the cloud silently.
 *****************************************************************************/

#ifndef LSLIDAR_UNIFIED_POINT_HPP
#define LSLIDAR_UNIFIED_POINT_HPP

#include <cstdint>
#include <cstring>
#include <string>

#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "lslidar_driver/lslidar_pointcloud.hpp"

namespace lslidar_driver {

    // detect::common::IntensityClass::NONE -- LSLidar intensity is raw,
    // uncalibrated RSSI, which tells dust_filter to skip its Livox tag/intensity
    // stages for this cloud.
    constexpr uint8_t kIntensityClassNone = 0;

#pragma pack(push, 1)
    struct UnifiedPoint {
        float    x;               //  0
        float    y;               //  4
        float    z;               //  8
        float    intensity;       // 12
        uint16_t ring;            // 16
        float    time;            // 18
        uint8_t  tag;             // 22
        uint8_t  intensity_class; // 23
    };
#pragma pack(pop)
    static_assert(sizeof(UnifiedPoint) == 24, "UnifiedPoint must be exactly 24 bytes");

    inline void toUnifiedCloud(const pcl::PointCloud<PointXYZIRT> &in,
                               const std::string &frame_id,
                               sensor_msgs::msg::PointCloud2 &out) {
        const uint32_t n = static_cast<uint32_t>(in.points.size());
        out.header.frame_id = frame_id;
        out.height = 1;
        out.width = n;
        out.is_dense = false;  // x/y/z copied verbatim; may contain NaN/Inf
        out.is_bigendian = false;
        out.point_step = sizeof(UnifiedPoint);
        out.row_step = out.point_step * n;

        out.fields.clear();
        auto addField = [&out](const char *name, uint32_t offset, uint8_t datatype) {
            sensor_msgs::msg::PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = datatype;
            f.count = 1;
            out.fields.push_back(f);
        };
        addField("x",               0,  sensor_msgs::msg::PointField::FLOAT32);
        addField("y",               4,  sensor_msgs::msg::PointField::FLOAT32);
        addField("z",               8,  sensor_msgs::msg::PointField::FLOAT32);
        addField("intensity",       12, sensor_msgs::msg::PointField::FLOAT32);
        addField("ring",            16, sensor_msgs::msg::PointField::UINT16);
        addField("time",            18, sensor_msgs::msg::PointField::FLOAT32);
        addField("tag",             22, sensor_msgs::msg::PointField::UINT8);
        addField("intensity_class", 23, sensor_msgs::msg::PointField::UINT8);

        out.data.resize(out.row_step);
        for (uint32_t i = 0; i < n; ++i) {
            const auto &src = in.points[i];
            UnifiedPoint dst;
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = static_cast<float>(src.time);
            dst.tag = 0;
            dst.intensity_class = kIntensityClassNone;
            std::memcpy(out.data.data() + i * sizeof(UnifiedPoint), &dst, sizeof(UnifiedPoint));
        }
    }

    // Time-based output cap, identical to sensor_adaptor's shouldPublish().
    // The threshold is 0.75 of the period rather than the whole period so a
    // frame arriving a hair early still passes: against a 10 Hz cap a 20 Hz
    // source (frames ~50 ms apart) is halved and a 10 Hz source (~100 ms) is
    // untouched. min_period_s <= 0 disables the cap.
    inline double minPublishPeriod(double max_rate_hz) {
        return max_rate_hz > 0.0 ? 0.75 / max_rate_hz : 0.0;
    }

    inline bool shouldPublish(double stamp_s, double &last_pub_s, double min_period_s) {
        const double dt = stamp_s - last_pub_s;
        // dt < 0 is a source clock stepping backwards, dt == 0 an unset stamp.
        // Publish in both cases rather than stalling the topic for good.
        if (min_period_s > 0.0 && dt > 0.0 && dt < min_period_s) {
            return false;
        }
        last_pub_s = stamp_s;
        return true;
    }

}  // namespace lslidar_driver

#endif  // LSLIDAR_UNIFIED_POINT_HPP
