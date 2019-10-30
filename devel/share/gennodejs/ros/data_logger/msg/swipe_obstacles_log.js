// Auto-generated. Do not edit!

// (in-package data_logger.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class swipe_obstacles_log {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.round = null;
      this.pose = null;
      this.odom = null;
      this.autoware_twist = null;
      this.ypspur_twist = null;
      this.brake = null;
      this.accel = null;
      this.shift = null;
      this.obstacle_id = null;
      this.detected_flag = null;
      this.pedestrian_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('round')) {
        this.round = initObj.round
      }
      else {
        this.round = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('odom')) {
        this.odom = initObj.odom
      }
      else {
        this.odom = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('autoware_twist')) {
        this.autoware_twist = initObj.autoware_twist
      }
      else {
        this.autoware_twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('ypspur_twist')) {
        this.ypspur_twist = initObj.ypspur_twist
      }
      else {
        this.ypspur_twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('brake')) {
        this.brake = initObj.brake
      }
      else {
        this.brake = 0.0;
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = 0.0;
      }
      if (initObj.hasOwnProperty('shift')) {
        this.shift = initObj.shift
      }
      else {
        this.shift = 0;
      }
      if (initObj.hasOwnProperty('obstacle_id')) {
        this.obstacle_id = initObj.obstacle_id
      }
      else {
        this.obstacle_id = 0;
      }
      if (initObj.hasOwnProperty('detected_flag')) {
        this.detected_flag = initObj.detected_flag
      }
      else {
        this.detected_flag = 0;
      }
      if (initObj.hasOwnProperty('pedestrian_flag')) {
        this.pedestrian_flag = initObj.pedestrian_flag
      }
      else {
        this.pedestrian_flag = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type swipe_obstacles_log
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [round]
    bufferOffset = _serializer.uint32(obj.round, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [odom]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.odom, buffer, bufferOffset);
    // Serialize message field [autoware_twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.autoware_twist, buffer, bufferOffset);
    // Serialize message field [ypspur_twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.ypspur_twist, buffer, bufferOffset);
    // Serialize message field [brake]
    bufferOffset = _serializer.float32(obj.brake, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = _serializer.float32(obj.accel, buffer, bufferOffset);
    // Serialize message field [shift]
    bufferOffset = _serializer.uint32(obj.shift, buffer, bufferOffset);
    // Serialize message field [obstacle_id]
    bufferOffset = _serializer.uint32(obj.obstacle_id, buffer, bufferOffset);
    // Serialize message field [detected_flag]
    bufferOffset = _serializer.uint32(obj.detected_flag, buffer, bufferOffset);
    // Serialize message field [pedestrian_flag]
    bufferOffset = _serializer.uint32(obj.pedestrian_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type swipe_obstacles_log
    let len;
    let data = new swipe_obstacles_log(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [round]
    data.round = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [odom]
    data.odom = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [autoware_twist]
    data.autoware_twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [ypspur_twist]
    data.ypspur_twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [brake]
    data.brake = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [shift]
    data.shift = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [obstacle_id]
    data.obstacle_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [detected_flag]
    data.detected_flag = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pedestrian_flag]
    data.pedestrian_flag = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 236;
  }

  static datatype() {
    // Returns string type for a message object
    return 'data_logger/swipe_obstacles_log';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '40f7308363d60b6e3d831771732fa8bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint32 round
    geometry_msgs/Pose pose
    geometry_msgs/Pose odom
    geometry_msgs/Twist autoware_twist
    geometry_msgs/Twist ypspur_twist
    float32 brake
    float32 accel
    uint32 shift
    uint32 obstacle_id
    uint32 detected_flag
    uint32 pedestrian_flag
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
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
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new swipe_obstacles_log(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.round !== undefined) {
      resolved.round = msg.round;
    }
    else {
      resolved.round = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.odom !== undefined) {
      resolved.odom = geometry_msgs.msg.Pose.Resolve(msg.odom)
    }
    else {
      resolved.odom = new geometry_msgs.msg.Pose()
    }

    if (msg.autoware_twist !== undefined) {
      resolved.autoware_twist = geometry_msgs.msg.Twist.Resolve(msg.autoware_twist)
    }
    else {
      resolved.autoware_twist = new geometry_msgs.msg.Twist()
    }

    if (msg.ypspur_twist !== undefined) {
      resolved.ypspur_twist = geometry_msgs.msg.Twist.Resolve(msg.ypspur_twist)
    }
    else {
      resolved.ypspur_twist = new geometry_msgs.msg.Twist()
    }

    if (msg.brake !== undefined) {
      resolved.brake = msg.brake;
    }
    else {
      resolved.brake = 0.0
    }

    if (msg.accel !== undefined) {
      resolved.accel = msg.accel;
    }
    else {
      resolved.accel = 0.0
    }

    if (msg.shift !== undefined) {
      resolved.shift = msg.shift;
    }
    else {
      resolved.shift = 0
    }

    if (msg.obstacle_id !== undefined) {
      resolved.obstacle_id = msg.obstacle_id;
    }
    else {
      resolved.obstacle_id = 0
    }

    if (msg.detected_flag !== undefined) {
      resolved.detected_flag = msg.detected_flag;
    }
    else {
      resolved.detected_flag = 0
    }

    if (msg.pedestrian_flag !== undefined) {
      resolved.pedestrian_flag = msg.pedestrian_flag;
    }
    else {
      resolved.pedestrian_flag = 0
    }

    return resolved;
    }
};

module.exports = swipe_obstacles_log;
