; Auto-generated. Do not edit!


(cl:in-package rpicar-msg)


;//! \htmlinclude telemetry.msg.html

(cl:defclass <telemetry> (roslisp-msg-protocol:ros-message)
  ((US1
    :reader US1
    :initarg :US1
    :type sensor_msgs-msg:Range
    :initform (cl:make-instance 'sensor_msgs-msg:Range))
   (US2
    :reader US2
    :initarg :US2
    :type sensor_msgs-msg:Range
    :initform (cl:make-instance 'sensor_msgs-msg:Range))
   (Battery
    :reader Battery
    :initarg :Battery
    :type sensor_msgs-msg:BatteryState
    :initform (cl:make-instance 'sensor_msgs-msg:BatteryState))
   (IMU
    :reader IMU
    :initarg :IMU
    :type sensor_msgs-msg:Imu
    :initform (cl:make-instance 'sensor_msgs-msg:Imu))
   (IMU_temp
    :reader IMU_temp
    :initarg :IMU_temp
    :type sensor_msgs-msg:Temperature
    :initform (cl:make-instance 'sensor_msgs-msg:Temperature))
   (IMU_mag
    :reader IMU_mag
    :initarg :IMU_mag
    :type sensor_msgs-msg:MagneticField
    :initform (cl:make-instance 'sensor_msgs-msg:MagneticField))
   (VO
    :reader VO
    :initarg :VO
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (EKF_pose
    :reader EKF_pose
    :initarg :EKF_pose
    :type geometry_msgs-msg:PoseWithCovarianceStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovarianceStamped)))
)

(cl:defclass telemetry (<telemetry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <telemetry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'telemetry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rpicar-msg:<telemetry> is deprecated: use rpicar-msg:telemetry instead.")))

(cl:ensure-generic-function 'US1-val :lambda-list '(m))
(cl:defmethod US1-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:US1-val is deprecated.  Use rpicar-msg:US1 instead.")
  (US1 m))

(cl:ensure-generic-function 'US2-val :lambda-list '(m))
(cl:defmethod US2-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:US2-val is deprecated.  Use rpicar-msg:US2 instead.")
  (US2 m))

(cl:ensure-generic-function 'Battery-val :lambda-list '(m))
(cl:defmethod Battery-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:Battery-val is deprecated.  Use rpicar-msg:Battery instead.")
  (Battery m))

(cl:ensure-generic-function 'IMU-val :lambda-list '(m))
(cl:defmethod IMU-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:IMU-val is deprecated.  Use rpicar-msg:IMU instead.")
  (IMU m))

(cl:ensure-generic-function 'IMU_temp-val :lambda-list '(m))
(cl:defmethod IMU_temp-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:IMU_temp-val is deprecated.  Use rpicar-msg:IMU_temp instead.")
  (IMU_temp m))

(cl:ensure-generic-function 'IMU_mag-val :lambda-list '(m))
(cl:defmethod IMU_mag-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:IMU_mag-val is deprecated.  Use rpicar-msg:IMU_mag instead.")
  (IMU_mag m))

(cl:ensure-generic-function 'VO-val :lambda-list '(m))
(cl:defmethod VO-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:VO-val is deprecated.  Use rpicar-msg:VO instead.")
  (VO m))

(cl:ensure-generic-function 'EKF_pose-val :lambda-list '(m))
(cl:defmethod EKF_pose-val ((m <telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rpicar-msg:EKF_pose-val is deprecated.  Use rpicar-msg:EKF_pose instead.")
  (EKF_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <telemetry>) ostream)
  "Serializes a message object of type '<telemetry>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'US1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'US2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Battery) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'IMU) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'IMU_temp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'IMU_mag) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'VO) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'EKF_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <telemetry>) istream)
  "Deserializes a message object of type '<telemetry>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'US1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'US2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Battery) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'IMU) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'IMU_temp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'IMU_mag) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'VO) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'EKF_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<telemetry>)))
  "Returns string type for a message object of type '<telemetry>"
  "rpicar/telemetry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'telemetry)))
  "Returns string type for a message object of type 'telemetry"
  "rpicar/telemetry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<telemetry>)))
  "Returns md5sum for a message object of type '<telemetry>"
  "82f6d27977b146e068901e86d89cdaa2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'telemetry)))
  "Returns md5sum for a message object of type 'telemetry"
  "82f6d27977b146e068901e86d89cdaa2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<telemetry>)))
  "Returns full string definition for message of type '<telemetry>"
  (cl:format cl:nil "sensor_msgs/Range US1~%sensor_msgs/Range US2~%sensor_msgs/BatteryState Battery~%sensor_msgs/Imu IMU~%sensor_msgs/Temperature IMU_temp~%sensor_msgs/MagneticField IMU_mag~%nav_msgs/Odometry VO~%geometry_msgs/PoseWithCovarianceStamped EKF_pose~%~%================================================================================~%MSG: sensor_msgs/Range~%# Single range reading from an active ranger that emits energy and reports~%# one range reading that is valid along an arc at the distance measured. ~%# This message is  not appropriate for laser scanners. See the LaserScan~%# message if you are working with a laser scanner.~%~%# This message also can represent a fixed-distance (binary) ranger.  This~%# sensor will have min_range===max_range===distance of detection.~%# These sensors follow REP 117 and will output -Inf if the object is detected~%# and +Inf if the object is outside of the detection range.~%~%Header header           # timestamp in the header is the time the ranger~%                        # returned the distance reading~%~%# Radiation type enums~%# If you want a value added to this list, send an email to the ros-users list~%uint8 ULTRASOUND=0~%uint8 INFRARED=1~%~%uint8 radiation_type    # the type of radiation used by the sensor~%                        # (sound, IR, etc) [enum]~%~%float32 field_of_view   # the size of the arc that the distance reading is~%                        # valid for [rad]~%                        # the object causing the range reading may have~%                        # been anywhere within -field_of_view/2 and~%                        # field_of_view/2 at the measured range. ~%                        # 0 angle corresponds to the x-axis of the sensor.~%~%float32 min_range       # minimum range value [m]~%float32 max_range       # maximum range value [m]~%                        # Fixed distance rangers require min_range==max_range~%~%float32 range           # range data [m]~%                        # (Note: values < range_min or > range_max~%                        # should be discarded)~%                        # Fixed distance rangers only output -Inf or +Inf.~%                        # -Inf represents a detection within fixed distance.~%                        # (Detection too close to the sensor to quantify)~%                        # +Inf represents no detection within the fixed distance.~%                        # (Object out of range)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/BatteryState~%~%# Constants are chosen to match the enums in the linux kernel~%# defined in include/linux/power_supply.h as of version 3.7~%# The one difference is for style reasons the constants are~%# all uppercase not mixed case.~%~%# Power supply status constants~%uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0~%uint8 POWER_SUPPLY_STATUS_CHARGING = 1~%uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2~%uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3~%uint8 POWER_SUPPLY_STATUS_FULL = 4~%~%# Power supply health constants~%uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0~%uint8 POWER_SUPPLY_HEALTH_GOOD = 1~%uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2~%uint8 POWER_SUPPLY_HEALTH_DEAD = 3~%uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4~%uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5~%uint8 POWER_SUPPLY_HEALTH_COLD = 6~%uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7~%uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8~%~%# Power supply technology (chemistry) constants~%uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0~%uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1~%uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2~%uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3~%uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4~%uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5~%uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6~%~%Header  header~%float32 voltage          # Voltage in Volts (Mandatory)~%float32 current          # Negative when discharging (A)  (If unmeasured NaN)~%float32 charge           # Current charge in Ah  (If unmeasured NaN)~%float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)~%float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)~%float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)~%uint8   power_supply_status     # The charging status as reported. Values defined above~%uint8   power_supply_health     # The battery health metric. Values defined above~%uint8   power_supply_technology # The battery chemistry. Values defined above~%bool    present          # True if the battery is present~%~%float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack~%                         # If individual voltages unknown but number of cells known set each to NaN~%string location          # The location into which the battery is inserted. (slot number or plug)~%string serial_number     # The best approximation of the battery serial number~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/Temperature~% # Single temperature reading.~%~% Header header           # timestamp is the time the temperature was measured~%                         # frame_id is the location of the temperature reading~%~% float64 temperature     # Measurement of the Temperature in Degrees Celsius~%~% float64 variance        # 0 is interpreted as variance unknown~%================================================================================~%MSG: sensor_msgs/MagneticField~% # Measurement of the Magnetic Field vector at a specific location.~%~% # If the covariance of the measurement is known, it should be filled in~% # (if all you know is the variance of each measurement, e.g. from the datasheet,~% #just put those along the diagonal)~% # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",~% # and to use the data a covariance will have to be assumed or gotten from some~% # other source~%~%~% Header header                        # timestamp is the time the~%                                      # field was measured~%                                      # frame_id is the location and orientation~%                                      # of the field measurement~%~% geometry_msgs/Vector3 magnetic_field # x, y, and z components of the~%                                      # field vector in Tesla~%                                      # If your sensor does not output 3 axes,~%                                      # put NaNs in the components not reported.~%~% float64[9] magnetic_field_covariance # Row major about x, y, z axes~%                                      # 0 is interpreted as variance unknown~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'telemetry)))
  "Returns full string definition for message of type 'telemetry"
  (cl:format cl:nil "sensor_msgs/Range US1~%sensor_msgs/Range US2~%sensor_msgs/BatteryState Battery~%sensor_msgs/Imu IMU~%sensor_msgs/Temperature IMU_temp~%sensor_msgs/MagneticField IMU_mag~%nav_msgs/Odometry VO~%geometry_msgs/PoseWithCovarianceStamped EKF_pose~%~%================================================================================~%MSG: sensor_msgs/Range~%# Single range reading from an active ranger that emits energy and reports~%# one range reading that is valid along an arc at the distance measured. ~%# This message is  not appropriate for laser scanners. See the LaserScan~%# message if you are working with a laser scanner.~%~%# This message also can represent a fixed-distance (binary) ranger.  This~%# sensor will have min_range===max_range===distance of detection.~%# These sensors follow REP 117 and will output -Inf if the object is detected~%# and +Inf if the object is outside of the detection range.~%~%Header header           # timestamp in the header is the time the ranger~%                        # returned the distance reading~%~%# Radiation type enums~%# If you want a value added to this list, send an email to the ros-users list~%uint8 ULTRASOUND=0~%uint8 INFRARED=1~%~%uint8 radiation_type    # the type of radiation used by the sensor~%                        # (sound, IR, etc) [enum]~%~%float32 field_of_view   # the size of the arc that the distance reading is~%                        # valid for [rad]~%                        # the object causing the range reading may have~%                        # been anywhere within -field_of_view/2 and~%                        # field_of_view/2 at the measured range. ~%                        # 0 angle corresponds to the x-axis of the sensor.~%~%float32 min_range       # minimum range value [m]~%float32 max_range       # maximum range value [m]~%                        # Fixed distance rangers require min_range==max_range~%~%float32 range           # range data [m]~%                        # (Note: values < range_min or > range_max~%                        # should be discarded)~%                        # Fixed distance rangers only output -Inf or +Inf.~%                        # -Inf represents a detection within fixed distance.~%                        # (Detection too close to the sensor to quantify)~%                        # +Inf represents no detection within the fixed distance.~%                        # (Object out of range)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/BatteryState~%~%# Constants are chosen to match the enums in the linux kernel~%# defined in include/linux/power_supply.h as of version 3.7~%# The one difference is for style reasons the constants are~%# all uppercase not mixed case.~%~%# Power supply status constants~%uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0~%uint8 POWER_SUPPLY_STATUS_CHARGING = 1~%uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2~%uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3~%uint8 POWER_SUPPLY_STATUS_FULL = 4~%~%# Power supply health constants~%uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0~%uint8 POWER_SUPPLY_HEALTH_GOOD = 1~%uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2~%uint8 POWER_SUPPLY_HEALTH_DEAD = 3~%uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4~%uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5~%uint8 POWER_SUPPLY_HEALTH_COLD = 6~%uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7~%uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8~%~%# Power supply technology (chemistry) constants~%uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0~%uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1~%uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2~%uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3~%uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4~%uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5~%uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6~%~%Header  header~%float32 voltage          # Voltage in Volts (Mandatory)~%float32 current          # Negative when discharging (A)  (If unmeasured NaN)~%float32 charge           # Current charge in Ah  (If unmeasured NaN)~%float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)~%float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)~%float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)~%uint8   power_supply_status     # The charging status as reported. Values defined above~%uint8   power_supply_health     # The battery health metric. Values defined above~%uint8   power_supply_technology # The battery chemistry. Values defined above~%bool    present          # True if the battery is present~%~%float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack~%                         # If individual voltages unknown but number of cells known set each to NaN~%string location          # The location into which the battery is inserted. (slot number or plug)~%string serial_number     # The best approximation of the battery serial number~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/Temperature~% # Single temperature reading.~%~% Header header           # timestamp is the time the temperature was measured~%                         # frame_id is the location of the temperature reading~%~% float64 temperature     # Measurement of the Temperature in Degrees Celsius~%~% float64 variance        # 0 is interpreted as variance unknown~%================================================================================~%MSG: sensor_msgs/MagneticField~% # Measurement of the Magnetic Field vector at a specific location.~%~% # If the covariance of the measurement is known, it should be filled in~% # (if all you know is the variance of each measurement, e.g. from the datasheet,~% #just put those along the diagonal)~% # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",~% # and to use the data a covariance will have to be assumed or gotten from some~% # other source~%~%~% Header header                        # timestamp is the time the~%                                      # field was measured~%                                      # frame_id is the location and orientation~%                                      # of the field measurement~%~% geometry_msgs/Vector3 magnetic_field # x, y, and z components of the~%                                      # field vector in Tesla~%                                      # If your sensor does not output 3 axes,~%                                      # put NaNs in the components not reported.~%~% float64[9] magnetic_field_covariance # Row major about x, y, z axes~%                                      # 0 is interpreted as variance unknown~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <telemetry>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'US1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'US2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Battery))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'IMU))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'IMU_temp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'IMU_mag))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'VO))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'EKF_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <telemetry>))
  "Converts a ROS message object to a list"
  (cl:list 'telemetry
    (cl:cons ':US1 (US1 msg))
    (cl:cons ':US2 (US2 msg))
    (cl:cons ':Battery (Battery msg))
    (cl:cons ':IMU (IMU msg))
    (cl:cons ':IMU_temp (IMU_temp msg))
    (cl:cons ':IMU_mag (IMU_mag msg))
    (cl:cons ':VO (VO msg))
    (cl:cons ':EKF_pose (EKF_pose msg))
))
