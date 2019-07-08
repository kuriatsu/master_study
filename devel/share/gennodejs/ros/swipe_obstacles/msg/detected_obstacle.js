// Auto-generated. Do not edit!

// (in-package swipe_obstacles.msg)


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

class detected_obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.managed_id = null;
      this.label = null;
      this.score = null;
      this.pose = null;
      this.shift_x = null;
      this.shift_y = null;
      this.round = null;
      this.detected_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('managed_id')) {
        this.managed_id = initObj.managed_id
      }
      else {
        this.managed_id = 0;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('shift_x')) {
        this.shift_x = initObj.shift_x
      }
      else {
        this.shift_x = 0.0;
      }
      if (initObj.hasOwnProperty('shift_y')) {
        this.shift_y = initObj.shift_y
      }
      else {
        this.shift_y = 0.0;
      }
      if (initObj.hasOwnProperty('round')) {
        this.round = initObj.round
      }
      else {
        this.round = 0;
      }
      if (initObj.hasOwnProperty('detected_time')) {
        this.detected_time = initObj.detected_time
      }
      else {
        this.detected_time = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type detected_obstacle
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [managed_id]
    bufferOffset = _serializer.uint32(obj.managed_id, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.float32(obj.score, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [shift_x]
    bufferOffset = _serializer.float32(obj.shift_x, buffer, bufferOffset);
    // Serialize message field [shift_y]
    bufferOffset = _serializer.float32(obj.shift_y, buffer, bufferOffset);
    // Serialize message field [round]
    bufferOffset = _serializer.uint32(obj.round, buffer, bufferOffset);
    // Serialize message field [detected_time]
    bufferOffset = _serializer.time(obj.detected_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type detected_obstacle
    let len;
    let data = new detected_obstacle(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [managed_id]
    data.managed_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [shift_x]
    data.shift_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [shift_y]
    data.shift_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [round]
    data.round = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [detected_time]
    data.detected_time = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.label.length;
    return length + 92;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swipe_obstacles/detected_obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9978e5bb58ee01f8e45fc2d8376b759a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint32 id
    uint32 managed_id
    string label
    float32 score
    geometry_msgs/Pose pose
    
    float32 shift_x
    float32 shift_y
    uint32 round
    time detected_time
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new detected_obstacle(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.managed_id !== undefined) {
      resolved.managed_id = msg.managed_id;
    }
    else {
      resolved.managed_id = 0
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.shift_x !== undefined) {
      resolved.shift_x = msg.shift_x;
    }
    else {
      resolved.shift_x = 0.0
    }

    if (msg.shift_y !== undefined) {
      resolved.shift_y = msg.shift_y;
    }
    else {
      resolved.shift_y = 0.0
    }

    if (msg.round !== undefined) {
      resolved.round = msg.round;
    }
    else {
      resolved.round = 0
    }

    if (msg.detected_time !== undefined) {
      resolved.detected_time = msg.detected_time;
    }
    else {
      resolved.detected_time = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = detected_obstacle;
