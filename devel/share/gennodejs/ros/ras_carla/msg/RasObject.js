// Auto-generated. Do not edit!

// (in-package ras_carla.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let derived_object_msgs = _finder('derived_object_msgs');

//-----------------------------------------------------------

class RasObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object = null;
      this.distance = null;
      this.shift_x = null;
      this.shift_y = null;
      this.is_front = null;
      this.importance = null;
    }
    else {
      if (initObj.hasOwnProperty('object')) {
        this.object = initObj.object
      }
      else {
        this.object = new derived_object_msgs.msg.Object();
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
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
      if (initObj.hasOwnProperty('is_front')) {
        this.is_front = initObj.is_front
      }
      else {
        this.is_front = false;
      }
      if (initObj.hasOwnProperty('importance')) {
        this.importance = initObj.importance
      }
      else {
        this.importance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RasObject
    // Serialize message field [object]
    bufferOffset = derived_object_msgs.msg.Object.serialize(obj.object, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [shift_x]
    bufferOffset = _serializer.float32(obj.shift_x, buffer, bufferOffset);
    // Serialize message field [shift_y]
    bufferOffset = _serializer.float32(obj.shift_y, buffer, bufferOffset);
    // Serialize message field [is_front]
    bufferOffset = _serializer.bool(obj.is_front, buffer, bufferOffset);
    // Serialize message field [importance]
    bufferOffset = _serializer.float32(obj.importance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RasObject
    let len;
    let data = new RasObject(null);
    // Deserialize message field [object]
    data.object = derived_object_msgs.msg.Object.deserialize(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [shift_x]
    data.shift_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [shift_y]
    data.shift_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_front]
    data.is_front = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [importance]
    data.importance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += derived_object_msgs.msg.Object.getMessageSize(object.object);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ras_carla/RasObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6890397792f930487f9106664fc3bee3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    derived_object_msgs/Object object
    float32 distance
    float32 shift_x
    float32 shift_y
    bool is_front
    float32 importance
    
    ================================================================================
    MSG: derived_object_msgs/Object
    # This represents a detected or tracked object with reference coordinate frame and timestamp.
    
    Header header
    
    # The id of the object (presumably from the detecting sensor).
    uint32 id
    
    # A Detected object is one which has been seen in at least one scan/frame of a sensor.
    # A Tracked object is one which has been correlated over multiple scans/frames of a sensor.
    # An object which is detected can only be assumed to have valid pose and shape properties.
    # An object which is tracked should also be assumed to have valid twist and accel properties.
    uint8 detection_level
    
    uint8 OBJECT_DETECTED=0
    uint8 OBJECT_TRACKED=1
    
    # A Classified object is one which has been identified as a certain object type.
    # Classified objects should have valid classification, classification_certainty, and classification_age properties.
    bool object_classified
    
    # The detected position and orientation of the object.
    geometry_msgs/Pose pose
    
    # The detected linear and angular velocities of the object.
    geometry_msgs/Twist twist
    
    # The detected linear and angular accelerations of the object.
    geometry_msgs/Accel accel
    
    # (OPTIONAL) The polygon defining the detection points at the outer edges of the object.
    geometry_msgs/Polygon polygon
    
    # A shape conforming to the outer bounding edges of the object.
    shape_msgs/SolidPrimitive shape
    
    # The type of classification given to this object.
    uint8 classification
    
    uint8 CLASSIFICATION_UNKNOWN=0
    uint8 CLASSIFICATION_UNKNOWN_SMALL=1
    uint8 CLASSIFICATION_UNKNOWN_MEDIUM=2
    uint8 CLASSIFICATION_UNKNOWN_BIG=3
    uint8 CLASSIFICATION_PEDESTRIAN=4
    uint8 CLASSIFICATION_BIKE=5
    uint8 CLASSIFICATION_CAR=6
    uint8 CLASSIFICATION_TRUCK=7
    uint8 CLASSIFICATION_MOTORCYCLE=8
    uint8 CLASSIFICATION_OTHER_VEHICLE=9
    uint8 CLASSIFICATION_BARRIER=10
    uint8 CLASSIFICATION_SIGN=11
    
    # The certainty of the classification from the originating sensor.
    # Higher value indicates greater certainty (MAX=255).
    # It is recommended that a native sensor value be scaled to 0-255 for interoperability.
    uint8 classification_certainty
    
    # The number of scans/frames from the sensor that this object has been classified as the current classification.
    uint32 classification_age
    
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
    ================================================================================
    MSG: geometry_msgs/Accel
    # This expresses acceleration in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Polygon
    #A specification of a polygon where the first and last points are assumed to be connected
    Point32[] points
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: shape_msgs/SolidPrimitive
    # Define box, sphere, cylinder, cone 
    # All shapes are defined to have their bounding boxes centered around 0,0,0.
    
    uint8 BOX=1
    uint8 SPHERE=2
    uint8 CYLINDER=3
    uint8 CONE=4
    
    # The type of the shape
    uint8 type
    
    
    # The dimensions of the shape
    float64[] dimensions
    
    # The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array
    
    # For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
    # sides of the box.
    uint8 BOX_X=0
    uint8 BOX_Y=1
    uint8 BOX_Z=2
    
    
    # For the SPHERE type, only one component is used, and it gives the radius of
    # the sphere.
    uint8 SPHERE_RADIUS=0
    
    
    # For the CYLINDER and CONE types, the center line is oriented along
    # the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
    # of dimensions gives the height of the cylinder (cone).  The
    # CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
    # radius of the base of the cylinder (cone).  Cone and cylinder
    # primitives are defined to be circular. The tip of the cone is
    # pointing up, along +Z axis.
    
    uint8 CYLINDER_HEIGHT=0
    uint8 CYLINDER_RADIUS=1
    
    uint8 CONE_HEIGHT=0
    uint8 CONE_RADIUS=1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RasObject(null);
    if (msg.object !== undefined) {
      resolved.object = derived_object_msgs.msg.Object.Resolve(msg.object)
    }
    else {
      resolved.object = new derived_object_msgs.msg.Object()
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
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

    if (msg.is_front !== undefined) {
      resolved.is_front = msg.is_front;
    }
    else {
      resolved.is_front = false
    }

    if (msg.importance !== undefined) {
      resolved.importance = msg.importance;
    }
    else {
      resolved.importance = 0.0
    }

    return resolved;
    }
};

module.exports = RasObject;
