// Auto-generated. Do not edit!

// (in-package rpicar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let geometry_msgs = _finder('geometry_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class telemetry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.US1 = null;
      this.US2 = null;
      this.Battery = null;
      this.IMU = null;
      this.IMU_temp = null;
      this.IMU_mag = null;
      this.VO = null;
      this.EKF_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('US1')) {
        this.US1 = initObj.US1
      }
      else {
        this.US1 = new sensor_msgs.msg.Range();
      }
      if (initObj.hasOwnProperty('US2')) {
        this.US2 = initObj.US2
      }
      else {
        this.US2 = new sensor_msgs.msg.Range();
      }
      if (initObj.hasOwnProperty('Battery')) {
        this.Battery = initObj.Battery
      }
      else {
        this.Battery = new sensor_msgs.msg.BatteryState();
      }
      if (initObj.hasOwnProperty('IMU')) {
        this.IMU = initObj.IMU
      }
      else {
        this.IMU = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('IMU_temp')) {
        this.IMU_temp = initObj.IMU_temp
      }
      else {
        this.IMU_temp = new sensor_msgs.msg.Temperature();
      }
      if (initObj.hasOwnProperty('IMU_mag')) {
        this.IMU_mag = initObj.IMU_mag
      }
      else {
        this.IMU_mag = new sensor_msgs.msg.MagneticField();
      }
      if (initObj.hasOwnProperty('VO')) {
        this.VO = initObj.VO
      }
      else {
        this.VO = new nav_msgs.msg.Odometry();
      }
      if (initObj.hasOwnProperty('EKF_pose')) {
        this.EKF_pose = initObj.EKF_pose
      }
      else {
        this.EKF_pose = new geometry_msgs.msg.PoseWithCovarianceStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type telemetry
    // Serialize message field [US1]
    bufferOffset = sensor_msgs.msg.Range.serialize(obj.US1, buffer, bufferOffset);
    // Serialize message field [US2]
    bufferOffset = sensor_msgs.msg.Range.serialize(obj.US2, buffer, bufferOffset);
    // Serialize message field [Battery]
    bufferOffset = sensor_msgs.msg.BatteryState.serialize(obj.Battery, buffer, bufferOffset);
    // Serialize message field [IMU]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.IMU, buffer, bufferOffset);
    // Serialize message field [IMU_temp]
    bufferOffset = sensor_msgs.msg.Temperature.serialize(obj.IMU_temp, buffer, bufferOffset);
    // Serialize message field [IMU_mag]
    bufferOffset = sensor_msgs.msg.MagneticField.serialize(obj.IMU_mag, buffer, bufferOffset);
    // Serialize message field [VO]
    bufferOffset = nav_msgs.msg.Odometry.serialize(obj.VO, buffer, bufferOffset);
    // Serialize message field [EKF_pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovarianceStamped.serialize(obj.EKF_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type telemetry
    let len;
    let data = new telemetry(null);
    // Deserialize message field [US1]
    data.US1 = sensor_msgs.msg.Range.deserialize(buffer, bufferOffset);
    // Deserialize message field [US2]
    data.US2 = sensor_msgs.msg.Range.deserialize(buffer, bufferOffset);
    // Deserialize message field [Battery]
    data.Battery = sensor_msgs.msg.BatteryState.deserialize(buffer, bufferOffset);
    // Deserialize message field [IMU]
    data.IMU = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [IMU_temp]
    data.IMU_temp = sensor_msgs.msg.Temperature.deserialize(buffer, bufferOffset);
    // Deserialize message field [IMU_mag]
    data.IMU_mag = sensor_msgs.msg.MagneticField.deserialize(buffer, bufferOffset);
    // Deserialize message field [VO]
    data.VO = nav_msgs.msg.Odometry.deserialize(buffer, bufferOffset);
    // Deserialize message field [EKF_pose]
    data.EKF_pose = geometry_msgs.msg.PoseWithCovarianceStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Range.getMessageSize(object.US1);
    length += sensor_msgs.msg.Range.getMessageSize(object.US2);
    length += sensor_msgs.msg.BatteryState.getMessageSize(object.Battery);
    length += sensor_msgs.msg.Imu.getMessageSize(object.IMU);
    length += sensor_msgs.msg.Temperature.getMessageSize(object.IMU_temp);
    length += sensor_msgs.msg.MagneticField.getMessageSize(object.IMU_mag);
    length += nav_msgs.msg.Odometry.getMessageSize(object.VO);
    length += geometry_msgs.msg.PoseWithCovarianceStamped.getMessageSize(object.EKF_pose);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rpicar/telemetry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82f6d27977b146e068901e86d89cdaa2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Range US1
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
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
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
     # A covariance matrix of all zeros will be interpreted as "covariance unknown",
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new telemetry(null);
    if (msg.US1 !== undefined) {
      resolved.US1 = sensor_msgs.msg.Range.Resolve(msg.US1)
    }
    else {
      resolved.US1 = new sensor_msgs.msg.Range()
    }

    if (msg.US2 !== undefined) {
      resolved.US2 = sensor_msgs.msg.Range.Resolve(msg.US2)
    }
    else {
      resolved.US2 = new sensor_msgs.msg.Range()
    }

    if (msg.Battery !== undefined) {
      resolved.Battery = sensor_msgs.msg.BatteryState.Resolve(msg.Battery)
    }
    else {
      resolved.Battery = new sensor_msgs.msg.BatteryState()
    }

    if (msg.IMU !== undefined) {
      resolved.IMU = sensor_msgs.msg.Imu.Resolve(msg.IMU)
    }
    else {
      resolved.IMU = new sensor_msgs.msg.Imu()
    }

    if (msg.IMU_temp !== undefined) {
      resolved.IMU_temp = sensor_msgs.msg.Temperature.Resolve(msg.IMU_temp)
    }
    else {
      resolved.IMU_temp = new sensor_msgs.msg.Temperature()
    }

    if (msg.IMU_mag !== undefined) {
      resolved.IMU_mag = sensor_msgs.msg.MagneticField.Resolve(msg.IMU_mag)
    }
    else {
      resolved.IMU_mag = new sensor_msgs.msg.MagneticField()
    }

    if (msg.VO !== undefined) {
      resolved.VO = nav_msgs.msg.Odometry.Resolve(msg.VO)
    }
    else {
      resolved.VO = new nav_msgs.msg.Odometry()
    }

    if (msg.EKF_pose !== undefined) {
      resolved.EKF_pose = geometry_msgs.msg.PoseWithCovarianceStamped.Resolve(msg.EKF_pose)
    }
    else {
      resolved.EKF_pose = new geometry_msgs.msg.PoseWithCovarianceStamped()
    }

    return resolved;
    }
};

module.exports = telemetry;
