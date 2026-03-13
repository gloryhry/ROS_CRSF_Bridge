#ifndef CRSF_CONTROL_CRSF_TELEMETRY_DECODER_H
#define CRSF_CONTROL_CRSF_TELEMETRY_DECODER_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace crsf_control {

class TelemetryDecoder {
public:
    static constexpr uint8_t CRSF_FRAMETYPE_GPS = 0x02;
    static constexpr uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08;
    static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
    static constexpr uint8_t CRSF_FRAMETYPE_ATTITUDE = 0x1E;
    static constexpr uint8_t CRSF_FRAMETYPE_FLIGHT_MODE = 0x21;

    static constexpr size_t GPS_PAYLOAD_SIZE = 15;
    static constexpr size_t BATTERY_SENSOR_PAYLOAD_SIZE = 8;
    static constexpr size_t LINK_STATISTICS_PAYLOAD_SIZE = 10;
    static constexpr size_t ATTITUDE_PAYLOAD_SIZE = 6;

    struct GpsData {
        int32_t latitude_e7 = 0;
        int32_t longitude_e7 = 0;

        double latitude_deg = 0.0;
        double longitude_deg = 0.0;

        double groundspeed_kmh = 0.0;
        double groundspeed_mps = 0.0;

        double heading_deg = 0.0;
        double altitude_m = 0.0;

        uint8_t satellites = 0;
    };

    struct BatteryData {
        double voltage_v = 0.0;
        double current_a = 0.0;
        uint32_t capacity_used_mah = 0;
        uint8_t remaining_percent = 0;
    };

    struct LinkStatisticsData {
        uint8_t uplink_rssi_1 = 0;
        uint8_t uplink_rssi_2 = 0;
        uint8_t uplink_link_quality = 0;
        int8_t uplink_snr = 0;
        uint8_t active_antenna = 0;
        uint8_t rf_mode = 0;
        uint8_t uplink_tx_power = 0;

        uint8_t downlink_rssi = 0;
        uint8_t downlink_link_quality = 0;
        int8_t downlink_snr = 0;
    };

    struct AttitudeData {
        double pitch_rad = 0.0;
        double roll_rad = 0.0;
        double yaw_rad = 0.0;

        double pitch_deg = 0.0;
        double roll_deg = 0.0;
        double yaw_deg = 0.0;
    };

    struct FlightModeData {
        std::string mode;
    };

    static uint16_t be16(const uint8_t* p);
    static int16_t beI16(const uint8_t* p);
    static uint32_t be24(const uint8_t* p);
    static uint32_t be32(const uint8_t* p);
    static int32_t beI32(const uint8_t* p);

    static bool decodeGps(const uint8_t* payload, size_t len, GpsData* out);
    static bool decodeBatterySensor(const uint8_t* payload, size_t len, BatteryData* out);
    static bool decodeLinkStatistics(const uint8_t* payload, size_t len, LinkStatisticsData* out);
    static bool decodeAttitude(const uint8_t* payload, size_t len, AttitudeData* out);
    static bool decodeFlightMode(const uint8_t* payload, size_t len, FlightModeData* out);

private:
    static bool sanityCheckLatLon(int32_t lat_e7, int32_t lon_e7);
    static bool sanityCheckPercent(uint8_t percent);
};

}  // namespace crsf_control

#endif  // CRSF_CONTROL_CRSF_TELEMETRY_DECODER_H
