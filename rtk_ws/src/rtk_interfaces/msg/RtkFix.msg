# NMEA GGA fix-quality constants.
uint8 QUALITY_NO_FIX=0
uint8 QUALITY_SINGLE=1
uint8 QUALITY_DGNSS=2
uint8 QUALITY_PPS=3
uint8 QUALITY_RTK_FIXED=4
uint8 QUALITY_RTK_FLOAT=5
uint8 QUALITY_DEAD_RECKONING=6

# Measurement time and receiver coordinate frame.
std_msgs/Header header

# WGS-84 position. altitude_ellipsoid is height above the ellipsoid in metres.
float64 latitude
float64 longitude
float64 altitude_ellipsoid

# Original GGA height fields, in metres.
float64 height_msl
float64 geoid_separation

# Quality and diagnostic fields.
uint8 fix_quality
string status
bool position_valid
bool rtk_fixed
uint8 satellites
float64 hdop
float64 correction_age

# Cumulative RTCM bytes written to the receiver since node startup.
uint64 rtcm_bytes

# Original GPGGA/GNGGA sentence for consumers that need raw data.
string gga
