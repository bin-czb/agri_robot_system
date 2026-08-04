"""
Configuration for the AOA Relative Localization System (test phase).

CURRENT GEOMETRY (very important — differs from the UWB node):
  - AOA base station is on the GROUND (connected to a PC).
  - AOA Tag is on the UAV.
  - The AOA base and depth-camera/IMU rig are rigidly mounted.  The
    AOA body frame follows ROS FLU:
        X = forward
        Y = left
        Z = up
    Body X coincides with East only when base_yaw_enu = 0.  On a moving
    cart the AOA bearing is
    rotated into ENU with the fused ground-base heading.
  - /uav/utm_pose (RTK) still gives the UAV absolute position.
  - The AOA range is treated as a slant range from base → tag:
        dz_tag    = altitude_uav + Lz − base_height
        rho_tag   = sqrt(max(r² − dz_tag², 0))
        θ_enu     = θ_body + base_yaw_enu
        p_tag_rel = [ rho_tag·cosθ_enu , rho_tag·sinθ_enu , dz_tag ]
  - If the tag is not co-located with the RTK antenna, the node rotates
    the configured Tag→RTK lever arm into ENU and converts to the UAV/RTK
    reference point:
        p_uav_rel = p_tag_rel − R_yaw · L_body
        base_abs  = uav_abs − p_uav_rel
  - UAV attitude is used only for the optional UAV-side lever arm.
    Ground-base heading is a separate input and rotates the AOA bearing.

IMPORTANT:  AOA has its own calibration parameters here and deliberately
does NOT import anything from uwb_config.  Keep the two configs
independent so tuning the AOA path cannot accidentally affect UWB.
"""

# ============================================================
#  Serial link
# ============================================================
AOA_SERIAL_PORT = '/dev/ttyACM0'    # AOA base-station UART
AOA_BAUDRATE    = 115200
AOA_TIMEOUT     = 0.1                # serial.read() timeout (seconds)

# ============================================================
#  Mount / sign calibration  — MUST be calibrated on first use
# ============================================================
# These two parameters map the AOA board's raw azimuth to the BASE
# body frame.  Semantics:
#   theta_body_deg = wrap( AOA_AZIMUTH_SIGN * azimuth_raw_deg
#                          + AOA_MOUNT_YAW_OFFSET_DEG )
#
# Calibration procedure (base on ground, UAV carries the tag):
#   1. Put the tag on the sensor rig's forward axis.
#      → measurement[3] should be near 0 deg.
#   2. Put the tag on the sensor rig's left side.
#      → measurement[3] should be near +90 deg.
#   3. If either sign is wrong, flip AOA_AZIMUTH_SIGN.
#   4. If the bearing is rotated by a fixed offset, put it in
#      AOA_MOUNT_YAW_OFFSET_DEG.
AOA_MOUNT_YAW_OFFSET_DEG = 0.0
AOA_AZIMUTH_SIGN         = +1        # +1 or -1 (handles CW vs CCW)

# ============================================================
#  Ground-base heading source
# ============================================================
# Modes:
#   fixed_east        Legacy bench mode.  Uses AOA_BASE_FIXED_YAW_DEG.
#   odometry_relative Recommended cart mode.  Reads orientation from fused
#                     nav_msgs/Odometry and anchors its first yaw sample to
#                     AOA_BASE_INITIAL_HEADING_ENU_DEG.
#   imu_relative      Controlled fallback.  Reads sensor_msgs/Imu orientation
#                     and uses the same first-sample anchor.  Raw Orbbec IMU
#                     does NOT contain absolute yaw, so /camera/imu/data from
#                     Madgwick may be used only as a relative-yaw source.
AOA_BASE_HEADING_MODE = 'odometry_relative'
AOA_BASE_ODOMETRY_TOPIC = '/odometry/filtered'
AOA_BASE_IMU_TOPIC = '/camera/imu/data'

# Absolute ENU heading of the sensor rig when the selected relative heading
# source emits its first valid sample.  0 deg = East, +90 deg = North.
# Set this from a surveyed/map-aligned initial pose.  IMU alone cannot infer it.
AOA_BASE_INITIAL_HEADING_ENU_DEG = 0.0
AOA_BASE_HEADING_TIMEOUT = 0.5

# Legacy compatibility.  Existing fixed-East experiments can select
# base_heading_mode:=fixed_east without changing calibration files.
AOA_BASE_FRAME_FIXED_EAST = False
AOA_BASE_FIXED_YAW_DEG = 0.0

# Ground base_link -> AOA phase-centre lever arm in the rigid sensor-rig FLU
# frame [forward, left, up], metres.  Measure after final mounting.  The
# current z=0.19 m preserves the old TAG_HEIGHT geometry.
AOA_BASE_TO_ANTENNA_BODY_M = [0.0, 0.0, 0.19]

# Conservative single-observation uncertainty used by the standard covariance
# outputs.  It is propagated as sigma_xy^2 ~= sigma_r^2 + (rho*sigma_angle)^2.
AOA_POSITION_SIGMA_RANGE_M = 0.50
AOA_POSITION_SIGMA_AZIMUTH_DEG = 5.0
AOA_BASE_HEADING_SIGMA_DEG = 5.0

# ============================================================
#  Tag-to-RTK lever arm  (UAV side)
# ============================================================
# AOA reports the SLANT distance / bearing from the BASE to the TAG.
# /uav/utm_pose on the other hand reports the position of the
# flight-controller's RTK antenna.  If the tag and the RTK antenna
# are not at the same physical point on the UAV, every solved
# base_abs will carry a systematic error of magnitude ‖lever_arm‖
# and — as the UAV yaws, pitches or rolls — will drift in ENU.  To
# correct for it we publish:
#
#     tag_abs   = uav_rtk_abs + R_enu←flu(ψ, θ, φ) · AOA_TAG_OFFSET_BODY_M
#     base_abs  = tag_abs − p_uav_rel
#
# where AOA_TAG_OFFSET_BODY_M is the tag position relative to the
# RTK antenna, expressed in the UAV body FLU frame:
#     X = forward (nose)
#     Y = left
#     Z = up
# Units: metres.  Measure with a tape measure — no estimation needed.
# The Z component matters too: it feeds the slant→horizontal projection
# via dz_tag = altitude_uav + Lz − base_height, so a wrong sign here can
# bias the projected XY even if the horizontal offsets are perfect.
#
# NOTE on the rotation itself: the lever arm is rotated by the FULL
# 3-axis UAV attitude (yaw ψ + pitch θ + roll φ), not yaw only.  The
# node reconstructs the correct ENU←FLU rotation locally from the
# hybrid quaternion published by uav_gps_node.py by flipping the sign
# of pitch and roll (MAVLink FRD → ROS FLU).  This matters in flight:
# at 10° bank and Lz ≈ −0.6 m, the yaw-only approximation was biased
# by roughly 0.2 m in XY.
#
# Keep [0.0, 0.0, 0.0] when the tag and the RTK antenna are
# co-located (or close enough that the ~10 cm residual is dominated
# by AOA bearing noise).
AOA_TAG_OFFSET_BODY_M = [-0.01, -0.045, -0.60]

# ============================================================
#  Range calibration
# ============================================================
# Applied as:  r_cal = (r_raw - AOA_RANGE_BIAS_M) / AOA_RANGE_SCALE
AOA_RANGE_BIAS_M = 0.0
AOA_RANGE_SCALE  = 1.0

# Physics gate — frames outside this range are dropped.
AOA_MIN_RANGE = 0.2     # m
AOA_MAX_RANGE = 100.0   # m

# ============================================================
#  Link / measurement watchdog
# ============================================================
AOA_MEAS_TIMEOUT      = 1.0   # s without a valid 0x2001 → pause publishing
AOA_HEARTBEAT_TIMEOUT = 5.0   # s without a 0x2002 → log link stale (info only)

# ============================================================
#  Elevation handling
# ============================================================
# Spec lists Elevation as "reserved".  The parser always decodes it so
# the frame alignment and XOR stay correct, but the fusion layer does
# not use it unless this flag is True.  Keep False in the test stage.
AOA_USE_ELEVATION = False

# ============================================================
#  Light-weight smoothing (test stage only, no EKF)
# ============================================================
# EMA alpha applied each valid 0x2001 frame.  1.0 disables smoothing.
# Range goes through a simple scalar EMA.  Angle is smoothed in unit-
# vector space (cos, sin) to avoid 359°/1° wrap artefacts.
AOA_RANGE_ALPHA = 0.3
AOA_ANGLE_ALPHA = 0.3

# ============================================================
#  Sliding-window robust least-squares optimizer
# ============================================================
# Optional backend that keeps the original single-frame outputs
# untouched and publishes separate optimized poses.  It assumes the
# ground AOA base / chassis is static or slowly moving over the window.  The
# localization node disables it automatically in dynamic heading modes.
AOA_OPT_ENABLE = True

# Window size for accepted AOA observations.  Frames rejected by the
# existing gates do not enter this backend.
AOA_OPT_WINDOW_SIZE = 30
AOA_OPT_MIN_FRAMES  = 8

# SciPy least_squares iteration budget.
AOA_OPT_MAX_NFEV = 30

# Observation noise used to normalise residuals.
AOA_OPT_SIGMA_RHO_M          = 0.5
AOA_OPT_SIGMA_THETA_DEG      = 5.0
AOA_OPT_SIGMA_BETA_PRIOR_DEG = 15.0
AOA_OPT_BETA_BOUND_DEG       = 30.0

# Robust loss passed directly to scipy.optimize.least_squares.
# Valid values include: "linear", "soft_l1", "huber", "cauchy", "arctan".
AOA_OPT_LOSS    = "soft_l1"
AOA_OPT_F_SCALE = 1.0

# Keep True for normal operation so the optimizer works in local_origin
# coordinates and avoids UTM-sized numerical values.
AOA_OPT_REQUIRE_WORLD_ORIGIN = True

# ============================================================
#  Single-frame outlier gates
#  These run BEFORE the EMA update so that bad frames cannot
#  slowly poison the filter state.  Tune conservatively: gates
#  that are too tight at this stage are just as harmful as EMA
#  smearing a mirror flip into the output.
# ============================================================
# Max allowed ENU bearing rate after the ground heading has rotated the
# body observation into the world frame.  A typical pedestrian /
# slow vehicle target produces under ~60 deg/s of world bearing
# rate; multipath / mirror flips produce instantaneous jumps
# well above that.  180 deg/s leaves generous slack while still
# rejecting ~60° single-frame kicks at the expected 3–10 Hz rate.
AOA_ANGLE_JUMP_MAX_RATE_DPS = 180.0

# Absolute slack added on top of rate * dt.  Covers quantisation
# (1°/LSB on the wire) and small yaw estimation lag.
AOA_ANGLE_JUMP_SLACK_DEG    = 8.0

# If the previous angle sample is older than this, do not try to
# gate against it — just accept and treat the new frame as the
# seed.  Keeps the gate from locking us out after a stall.
AOA_ANGLE_JUMP_DT_MAX       = 0.5

# Max allowed radial rate.  AOA protocol reports cm/LSB at the
# native frame rate (~5 Hz), so 15 m/s between consecutive frames
# is way beyond any realistic target movement in our test setup.
AOA_RANGE_JUMP_MAX_MPS      = 15.0

# EMA singularity guard: after the unit-vector EMA step the
# resulting (cos,sin) magnitude must exceed this floor.  A value
# near 0 means the EMA has been fed two near-opposite angles,
# so the reconstructed bearing would be numerically meaningless.
# Reject the frame (do not commit the EMA update) when this
# happens.
AOA_ANGLE_EMA_MIN_MAG       = 0.30

# Force-recovery: if the gate keeps rejecting this many
# consecutive frames, assume the target really did jump (or the
# EMA is stuck in a bad state) and re-seed the EMA from the
# current raw measurement.  Prevents permanent lockout.
AOA_ANGLE_CONSEC_REJECT_RESET = 8

# NOTE: the UAV-side ATTITUDE-freshness warning threshold lives in
# systems/common/uav_gps_node.py (ATT_FRESH_WARN_SEC) because that
# node is shared with the UWB stack and should not import from the
# AOA package.  Document it here for discoverability only.

# ============================================================
#  Front/back disambiguation  (semi-automatic)
# ============================================================
# AOA modules with only an azimuth channel (no usable elevation)
# are inherently ambiguous between θ and θ+180° — they can't tell
# whether the tag is "in front of" or "behind" the boresight,
# especially at low geometric elevation where multipath dominates.
# Empirically we observe the module sometimes locks onto the WRONG
# branch for the first portion of a flight and only flips to the
# correct one once the UAV climbs high enough.
#
# To kill this in software we let the user supply a coarse PRIOR:
# the rough ENU direction from the UAV's first-fix point (i.e. the
# locked world_origin) to the base station.  Per-frame the node
# evaluates both candidate solutions, selects the one that points
# in the direction of this prior, and applies hysteresis so a
# single bad frame can't toggle the choice.
#
# Only DIRECTION matters; magnitude is ignored.
#
# Examples:
#   base due East of takeoff point  → [+1.0,  0.0]
#   base East-North of takeoff      → [+1.0, +1.0]
#   base due West of takeoff        → [-1.0,  0.0]
#   base due South of takeoff       → [ 0.0, -1.0]
#
# Set to None to disable disambiguation entirely (legacy
# behaviour: trust the raw azimuth as-is).
AOA_BASE_HINT_DIR_ENU = [-1.0, 0.0]

# Branch-switch hysteresis: a switch from the previously-selected
# branch is accepted only after this many consecutive frames vote
# for the new branch AND the new branch's prior-projection score
# beats the old one's by at least MARGIN_M.  Prevents single-frame
# noise from flipping the published base position.
AOA_DISAMBIG_HYSTERESIS_FRAMES = 5
AOA_DISAMBIG_MARGIN_M          = 0.5

# Geometric elevation gate (degrees).  Frames whose dz/rho geometry
# implies a line-of-sight elevation BELOW this threshold are rejected
# outright, because front/back ambiguity is essentially impossible to
# resolve there (the slant cone collapses to a near-horizontal sweep).
# Set to 0.0 to disable this gate.
AOA_MIN_GEOM_ELEVATION_DEG = 10.0

# ============================================================
#  Altitude sensor (semantics copied verbatim from uwb_config)
# ============================================================
ALTITUDE_STD     = 1.0      # Baro ≈ 1.0 m | mmWave radar ≈ 0.05 m
# Kept as TAG_HEIGHT to avoid churning callers, but under the new
# geometry (base on ground, tag on UAV) this parameter represents the
# BASE ANTENNA height above ground, i.e. the z offset from the local
# ground plane up to the AOA base-station's phase centre.  Used in:
#     z_rel = altitude_UAV - TAG_HEIGHT
# Default 0.0 means "base antenna is at ground level".
# Deprecated compatibility alias.  New code uses
# AOA_BASE_TO_ANTENNA_BODY_M[2] as the phase-centre height.
TAG_HEIGHT       = AOA_BASE_TO_ANTENNA_BODY_M[2]
ALTITUDE_TIMEOUT = 1.0      # s before dedicated altitude source is stale
POSE_TIMEOUT     = 1.0      # s before /uav/utm_pose is stale

# ============================================================
#  World / absolute frame (semantics copied from uwb_config)
# ============================================================
WORLD_ORIGIN_MODE = 'first_fix'  # 'first_fix' | 'fixed_utm'
FIXED_ORIGIN_E = 0.0
FIXED_ORIGIN_N = 0.0
FIXED_ORIGIN_Z = 0.0             # MUST share reference with /uav/utm_pose.z

ORIGIN_STABLE_FRAMES   = 10
ORIGIN_PAIRWISE_THRESH = 0.5     # Max jump between consecutive frames (m)
ORIGIN_WINDOW_RADIUS   = 0.5     # Max scatter from buffer mean (m)
