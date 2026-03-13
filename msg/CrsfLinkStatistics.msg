# CRSF link statistics (TBS Crossfire)
# Values are carried in the CRSF Link Statistics frame.

std_msgs/Header header

# Uplink
uint8 uplink_rssi_1
uint8 uplink_rssi_2
uint8 uplink_link_quality
int8 uplink_snr
uint8 active_antenna
uint8 rf_mode
uint8 uplink_tx_power

# Downlink
uint8 downlink_rssi
uint8 downlink_link_quality
int8 downlink_snr
