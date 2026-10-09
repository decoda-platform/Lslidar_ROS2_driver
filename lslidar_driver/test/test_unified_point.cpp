// Decoda: checks for unified_point.hpp. The byte offsets asserted here are the
// contract with detect-dev's detect::common::UnifiedPoint -- change them only
// together with that struct.

#include <cstddef>
#include <cstring>

#include <gtest/gtest.h>

#include "lslidar_driver/unified_point.hpp"

using lslidar_driver::minPublishPeriod;
using lslidar_driver::shouldPublish;
using lslidar_driver::UnifiedPoint;

TEST(UnifiedPoint, LayoutMatchesDetectDev) {
    EXPECT_EQ(sizeof(UnifiedPoint), 24u);
    EXPECT_EQ(offsetof(UnifiedPoint, x), 0u);
    EXPECT_EQ(offsetof(UnifiedPoint, y), 4u);
    EXPECT_EQ(offsetof(UnifiedPoint, z), 8u);
    EXPECT_EQ(offsetof(UnifiedPoint, intensity), 12u);
    EXPECT_EQ(offsetof(UnifiedPoint, ring), 16u);
    EXPECT_EQ(offsetof(UnifiedPoint, time), 18u);
    EXPECT_EQ(offsetof(UnifiedPoint, tag), 22u);
    EXPECT_EQ(offsetof(UnifiedPoint, intensity_class), 23u);
}

TEST(UnifiedPoint, ConvertsEveryField) {
    pcl::PointCloud<lslidar_driver::PointXYZIRT> in;
    lslidar_driver::PointXYZIRT p;
    p.x = 1.5f; p.y = -2.0f; p.z = 3.25f; p.intensity = 42.0f; p.ring = 127; p.time = 0.0125f;
    in.points.push_back(p);
    p.ring = 3;
    in.points.push_back(p);

    sensor_msgs::msg::PointCloud2 out;
    lslidar_driver::toUnifiedCloud(in, "lidar_front", out);

    EXPECT_EQ(out.header.frame_id, "lidar_front");
    EXPECT_EQ(out.width, 2u);
    EXPECT_EQ(out.height, 1u);
    EXPECT_EQ(out.point_step, 24u);
    EXPECT_EQ(out.row_step, 48u);
    ASSERT_EQ(out.data.size(), 48u);
    ASSERT_EQ(out.fields.size(), 8u);
    EXPECT_EQ(out.fields[5].name, "time");
    EXPECT_EQ(out.fields[5].offset, 18u);

    UnifiedPoint u;
    std::memcpy(&u, out.data.data(), sizeof(u));
    EXPECT_FLOAT_EQ(u.x, 1.5f);
    EXPECT_FLOAT_EQ(u.y, -2.0f);
    EXPECT_FLOAT_EQ(u.z, 3.25f);
    EXPECT_FLOAT_EQ(u.intensity, 42.0f);
    EXPECT_EQ(u.ring, 127);
    EXPECT_FLOAT_EQ(u.time, 0.0125f);
    EXPECT_EQ(u.tag, 0);
    EXPECT_EQ(u.intensity_class, lslidar_driver::kIntensityClassNone);

    std::memcpy(&u, out.data.data() + 24, sizeof(u));
    EXPECT_EQ(u.ring, 3);
}

namespace {
int countPublished(double rate_hz, double cap_hz, int n_frames) {
    double last = 0.0;
    int n = 0;
    const double t0 = 1000.0;
    for (int i = 0; i < n_frames; ++i) {
        if (shouldPublish(t0 + i / rate_hz, last, minPublishPeriod(cap_hz))) ++n;
    }
    return n;
}
}  // namespace

TEST(ShouldPublish, HalvesA20HzSource) {
    EXPECT_EQ(countPublished(20.0, 10.0, 200), 100);
}

TEST(ShouldPublish, PassesA10HzSourceUntouched) {
    EXPECT_EQ(countPublished(10.0, 10.0, 100), 100);
}

TEST(ShouldPublish, DisabledCapPassesEverything) {
    EXPECT_EQ(countPublished(20.0, 0.0, 200), 200);
}

TEST(ShouldPublish, ClockStepBackwardsReAnchors) {
    double last = 0.0;
    const double period = minPublishPeriod(10.0);
    EXPECT_TRUE(shouldPublish(100.0, last, period));
    EXPECT_TRUE(shouldPublish(50.0, last, period));
    EXPECT_FALSE(shouldPublish(50.05, last, period));
    EXPECT_TRUE(shouldPublish(50.1, last, period));
}

#include "lslidar_driver/difop_status.hpp"

TEST(DifopStatus, ParsesCh128x1Fields) {
    uint8_t difop[1206] = {};
    difop[8] = 0x04; difop[9] = 0xB1;  // 1201 rpm, as a 20 Hz lidar measures
    difop[44] = 1;                     // gPTP
    difop[51] = 0x01;                  // dust on
    const auto s = lslidar_driver::parseChDifopStatus(difop, true);
    EXPECT_EQ(s.motor_speed_rpm, 1201);
    EXPECT_EQ(s.time_service_mode, 1);
    EXPECT_EQ(s.dust_mode, 1);
}

TEST(DifopStatus, DustUnknownOnOtherModels) {
    uint8_t difop[1206] = {};
    difop[51] = 0x01;
    EXPECT_EQ(lslidar_driver::parseChDifopStatus(difop, false).dust_mode,
              lslidar_driver::kDustModeUnknown);
}
