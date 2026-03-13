#include "crsf_control/crsf_telemetry_decoder.h"

#include <cmath>

namespace crsf_control {

uint16_t TelemetryDecoder::be16(const uint8_t* p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) << 8) |
           static_cast<uint16_t>(p[1]);
}

int16_t TelemetryDecoder::beI16(const uint8_t* p)
{
    return static_cast<int16_t>(be16(p));
}

uint32_t TelemetryDecoder::be24(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 16) |
           (static_cast<uint32_t>(p[1]) << 8) |
           static_cast<uint32_t>(p[2]);
}

uint32_t TelemetryDecoder::be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

int32_t TelemetryDecoder::beI32(const uint8_t* p)
{
    return static_cast<int32_t>(be32(p));
}

bool TelemetryDecoder::decodeGps(const uint8_t* payload, size_t len, GpsData* out)
{
    if (payload == nullptr || out == nullptr) {
        return false;
    }
    if (len < GPS_PAYLOAD_SIZE) {
        return false;
    }

    const int32_t lat_e7 = beI32(payload + 0);
    const int32_t lon_e7 = beI32(payload + 4);

    if (!sanityCheckLatLon(lat_e7, lon_e7)) {
        return false;
    }

    const uint16_t groundspeed_raw = be16(payload + 8);
    const uint16_t heading_raw = be16(payload + 10);
    const uint16_t altitude_raw = be16(payload + 12);
    const uint8_t sats = payload[14];

    out->latitude_e7 = lat_e7;
    out->longitude_e7 = lon_e7;
    out->latitude_deg = static_cast<double>(lat_e7) * 1e-7;
    out->longitude_deg = static_cast<double>(lon_e7) * 1e-7;

    const double groundspeed_kmh = static_cast<double>(groundspeed_raw) / 10.0;
    out->groundspeed_kmh = groundspeed_kmh;
    out->groundspeed_mps = groundspeed_kmh * (1000.0 / 3600.0);

    out->heading_deg = static_cast<double>(heading_raw) / 100.0;
    out->altitude_m = static_cast<double>(altitude_raw) - 1000.0;
    out->satellites = sats;

    return true;
}

bool TelemetryDecoder::decodeBatterySensor(const uint8_t* payload, size_t len, BatteryData* out)
{
    if (payload == nullptr || out == nullptr) {
        return false;
    }
    if (len < BATTERY_SENSOR_PAYLOAD_SIZE) {
        return false;
    }

    const uint16_t voltage_raw = be16(payload + 0);
    const uint16_t current_raw = be16(payload + 2);
    const uint32_t capacity_raw = be24(payload + 4);
    const uint8_t remaining = payload[7];

    if (!sanityCheckPercent(remaining)) {
        return false;
    }

    out->voltage_v = static_cast<double>(voltage_raw) * 0.1;
    out->current_a = static_cast<double>(current_raw) * 0.1;
    out->capacity_used_mah = capacity_raw;
    out->remaining_percent = remaining;

    return true;
}

bool TelemetryDecoder::decodeLinkStatistics(const uint8_t* payload, size_t len, LinkStatisticsData* out)
{
    if (payload == nullptr || out == nullptr) {
        return false;
    }
    if (len < LINK_STATISTICS_PAYLOAD_SIZE) {
        return false;
    }

    out->uplink_rssi_1 = payload[0];
    out->uplink_rssi_2 = payload[1];
    out->uplink_link_quality = payload[2];
    out->uplink_snr = static_cast<int8_t>(payload[3]);
    out->active_antenna = payload[4];
    out->rf_mode = payload[5];
    out->uplink_tx_power = payload[6];
    out->downlink_rssi = payload[7];
    out->downlink_link_quality = payload[8];
    out->downlink_snr = static_cast<int8_t>(payload[9]);

    return true;
}

bool TelemetryDecoder::decodeAttitude(const uint8_t* payload, size_t len, AttitudeData* out)
{
    if (payload == nullptr || out == nullptr) {
        return false;
    }
    if (len < ATTITUDE_PAYLOAD_SIZE) {
        return false;
    }

    const int16_t pitch_raw = beI16(payload + 0);
    const int16_t roll_raw = beI16(payload + 2);
    const int16_t yaw_raw = beI16(payload + 4);

    const double pitch_rad = static_cast<double>(pitch_raw) * 1e-4;
    const double roll_rad = static_cast<double>(roll_raw) * 1e-4;
    const double yaw_rad = static_cast<double>(yaw_raw) * 1e-4;

    const double rad_to_deg = 180.0 / M_PI;
    const double pitch_deg = pitch_rad * rad_to_deg;
    const double roll_deg = roll_rad * rad_to_deg;
    const double yaw_deg = yaw_rad * rad_to_deg;

    if (pitch_deg < -180.0 || pitch_deg > 180.0) {
        return false;
    }
    if (roll_deg < -180.0 || roll_deg > 180.0) {
        return false;
    }
    if (yaw_deg < -180.0 || yaw_deg > 180.0) {
        return false;
    }

    out->pitch_rad = pitch_rad;
    out->roll_rad = roll_rad;
    out->yaw_rad = yaw_rad;

    out->pitch_deg = pitch_deg;
    out->roll_deg = roll_deg;
    out->yaw_deg = yaw_deg;

    return true;
}

bool TelemetryDecoder::decodeFlightMode(const uint8_t* payload, size_t len, FlightModeData* out)
{
    if (payload == nullptr || out == nullptr) {
        return false;
    }
    if (len == 0) {
        return false;
    }

    size_t n = 0;
    while (n < len && payload[n] != 0) {
        n += 1;
    }

    if (n == 0) {
        return false;
    }

    out->mode.assign(reinterpret_cast<const char*>(payload), n);
    return true;
}

bool TelemetryDecoder::sanityCheckLatLon(int32_t lat_e7, int32_t lon_e7)
{
    const int32_t maxLat = 90 * 10000000;
    const int32_t maxLon = 180 * 10000000;

    if (lat_e7 < -maxLat || lat_e7 > maxLat) {
        return false;
    }
    if (lon_e7 < -maxLon || lon_e7 > maxLon) {
        return false;
    }

    return true;
}

bool TelemetryDecoder::sanityCheckPercent(uint8_t percent)
{
    return percent <= 100;
}

}  // namespace crsf_control
