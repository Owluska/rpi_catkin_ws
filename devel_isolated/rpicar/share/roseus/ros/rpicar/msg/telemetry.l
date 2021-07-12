;; Auto-generated. Do not edit!


(when (boundp 'rpicar::telemetry)
  (if (not (find-package "RPICAR"))
    (make-package "RPICAR"))
  (shadow 'telemetry (find-package "RPICAR")))
(unless (find-package "RPICAR::TELEMETRY")
  (make-package "RPICAR::TELEMETRY"))

(in-package "ROS")
;;//! \htmlinclude telemetry.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass rpicar::telemetry
  :super ros::object
  :slots (_US1 _US2 _Battery _IMU _IMU_temp _IMU_mag _VO _EKF_pose ))

(defmethod rpicar::telemetry
  (:init
   (&key
    ((:US1 __US1) (instance sensor_msgs::Range :init))
    ((:US2 __US2) (instance sensor_msgs::Range :init))
    ((:Battery __Battery) (instance sensor_msgs::BatteryState :init))
    ((:IMU __IMU) (instance sensor_msgs::Imu :init))
    ((:IMU_temp __IMU_temp) (instance sensor_msgs::Temperature :init))
    ((:IMU_mag __IMU_mag) (instance sensor_msgs::MagneticField :init))
    ((:VO __VO) (instance nav_msgs::Odometry :init))
    ((:EKF_pose __EKF_pose) (instance geometry_msgs::PoseWithCovarianceStamped :init))
    )
   (send-super :init)
   (setq _US1 __US1)
   (setq _US2 __US2)
   (setq _Battery __Battery)
   (setq _IMU __IMU)
   (setq _IMU_temp __IMU_temp)
   (setq _IMU_mag __IMU_mag)
   (setq _VO __VO)
   (setq _EKF_pose __EKF_pose)
   self)
  (:US1
   (&rest __US1)
   (if (keywordp (car __US1))
       (send* _US1 __US1)
     (progn
       (if __US1 (setq _US1 (car __US1)))
       _US1)))
  (:US2
   (&rest __US2)
   (if (keywordp (car __US2))
       (send* _US2 __US2)
     (progn
       (if __US2 (setq _US2 (car __US2)))
       _US2)))
  (:Battery
   (&rest __Battery)
   (if (keywordp (car __Battery))
       (send* _Battery __Battery)
     (progn
       (if __Battery (setq _Battery (car __Battery)))
       _Battery)))
  (:IMU
   (&rest __IMU)
   (if (keywordp (car __IMU))
       (send* _IMU __IMU)
     (progn
       (if __IMU (setq _IMU (car __IMU)))
       _IMU)))
  (:IMU_temp
   (&rest __IMU_temp)
   (if (keywordp (car __IMU_temp))
       (send* _IMU_temp __IMU_temp)
     (progn
       (if __IMU_temp (setq _IMU_temp (car __IMU_temp)))
       _IMU_temp)))
  (:IMU_mag
   (&rest __IMU_mag)
   (if (keywordp (car __IMU_mag))
       (send* _IMU_mag __IMU_mag)
     (progn
       (if __IMU_mag (setq _IMU_mag (car __IMU_mag)))
       _IMU_mag)))
  (:VO
   (&rest __VO)
   (if (keywordp (car __VO))
       (send* _VO __VO)
     (progn
       (if __VO (setq _VO (car __VO)))
       _VO)))
  (:EKF_pose
   (&rest __EKF_pose)
   (if (keywordp (car __EKF_pose))
       (send* _EKF_pose __EKF_pose)
     (progn
       (if __EKF_pose (setq _EKF_pose (car __EKF_pose)))
       _EKF_pose)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Range _US1
    (send _US1 :serialization-length)
    ;; sensor_msgs/Range _US2
    (send _US2 :serialization-length)
    ;; sensor_msgs/BatteryState _Battery
    (send _Battery :serialization-length)
    ;; sensor_msgs/Imu _IMU
    (send _IMU :serialization-length)
    ;; sensor_msgs/Temperature _IMU_temp
    (send _IMU_temp :serialization-length)
    ;; sensor_msgs/MagneticField _IMU_mag
    (send _IMU_mag :serialization-length)
    ;; nav_msgs/Odometry _VO
    (send _VO :serialization-length)
    ;; geometry_msgs/PoseWithCovarianceStamped _EKF_pose
    (send _EKF_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Range _US1
       (send _US1 :serialize s)
     ;; sensor_msgs/Range _US2
       (send _US2 :serialize s)
     ;; sensor_msgs/BatteryState _Battery
       (send _Battery :serialize s)
     ;; sensor_msgs/Imu _IMU
       (send _IMU :serialize s)
     ;; sensor_msgs/Temperature _IMU_temp
       (send _IMU_temp :serialize s)
     ;; sensor_msgs/MagneticField _IMU_mag
       (send _IMU_mag :serialize s)
     ;; nav_msgs/Odometry _VO
       (send _VO :serialize s)
     ;; geometry_msgs/PoseWithCovarianceStamped _EKF_pose
       (send _EKF_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Range _US1
     (send _US1 :deserialize buf ptr-) (incf ptr- (send _US1 :serialization-length))
   ;; sensor_msgs/Range _US2
     (send _US2 :deserialize buf ptr-) (incf ptr- (send _US2 :serialization-length))
   ;; sensor_msgs/BatteryState _Battery
     (send _Battery :deserialize buf ptr-) (incf ptr- (send _Battery :serialization-length))
   ;; sensor_msgs/Imu _IMU
     (send _IMU :deserialize buf ptr-) (incf ptr- (send _IMU :serialization-length))
   ;; sensor_msgs/Temperature _IMU_temp
     (send _IMU_temp :deserialize buf ptr-) (incf ptr- (send _IMU_temp :serialization-length))
   ;; sensor_msgs/MagneticField _IMU_mag
     (send _IMU_mag :deserialize buf ptr-) (incf ptr- (send _IMU_mag :serialization-length))
   ;; nav_msgs/Odometry _VO
     (send _VO :deserialize buf ptr-) (incf ptr- (send _VO :serialization-length))
   ;; geometry_msgs/PoseWithCovarianceStamped _EKF_pose
     (send _EKF_pose :deserialize buf ptr-) (incf ptr- (send _EKF_pose :serialization-length))
   ;;
   self)
  )

(setf (get rpicar::telemetry :md5sum-) "82f6d27977b146e068901e86d89cdaa2")
(setf (get rpicar::telemetry :datatype-) "rpicar/telemetry")
(setf (get rpicar::telemetry :definition-)
      "sensor_msgs/Range US1
sensor_msgs/Range US2
sensor_msgs/BatteryState Battery
sensor_msgs/Imu IMU
sensor_msgs/Temperature IMU_temp
sensor_msgs/MagneticField IMU_mag
nav_msgs/Odometry VO
geometry_msgs/PoseWithCovarianceStamped EKF_pose

================================================================================
MSG: sensor_msgs/Range
# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.

# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

Header header           # timestamp in the header is the time the ranger
                        # returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range. 
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        # Fixed distance rangers require min_range==max_range

float32 range           # range data [m]
                        # (Note: values < range_min or > range_max
                        # should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/BatteryState

# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/Temperature
 # Single temperature reading.

 Header header           # timestamp is the time the temperature was measured
                         # frame_id is the location of the temperature reading

 float64 temperature     # Measurement of the Temperature in Degrees Celsius

 float64 variance        # 0 is interpreted as variance unknown
================================================================================
MSG: sensor_msgs/MagneticField
 # Measurement of the Magnetic Field vector at a specific location.

 # If the covariance of the measurement is known, it should be filled in
 # (if all you know is the variance of each measurement, e.g. from the datasheet,
 #just put those along the diagonal)
 # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",
 # and to use the data a covariance will have to be assumed or gotten from some
 # other source


 Header header                        # timestamp is the time the
                                      # field was measured
                                      # frame_id is the location and orientation
                                      # of the field measurement

 geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                      # field vector in Tesla
                                      # If your sensor does not output 3 axes,
                                      # put NaNs in the components not reported.

 float64[9] magnetic_field_covariance # Row major about x, y, z axes
                                      # 0 is interpreted as variance unknown
================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

")



(provide :rpicar/telemetry "82f6d27977b146e068901e86d89cdaa2")


