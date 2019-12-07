; Auto-generated. Do not edit!


(cl:in-package ras_carla-msg)


;//! \htmlinclude RasObject.msg.html

(cl:defclass <RasObject> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type derived_object_msgs-msg:Object
    :initform (cl:make-instance 'derived_object_msgs-msg:Object))
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (shift_x
    :reader shift_x
    :initarg :shift_x
    :type cl:float
    :initform 0.0)
   (shift_y
    :reader shift_y
    :initarg :shift_y
    :type cl:float
    :initform 0.0)
   (is_front
    :reader is_front
    :initarg :is_front
    :type cl:boolean
    :initform cl:nil)
   (importance
    :reader importance
    :initarg :importance
    :type cl:float
    :initform 0.0))
)

(cl:defclass RasObject (<RasObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RasObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RasObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ras_carla-msg:<RasObject> is deprecated: use ras_carla-msg:RasObject instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <RasObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ras_carla-msg:object-val is deprecated.  Use ras_carla-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <RasObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ras_carla-msg:distance-val is deprecated.  Use ras_carla-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'shift_x-val :lambda-list '(m))
(cl:defmethod shift_x-val ((m <RasObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ras_carla-msg:shift_x-val is deprecated.  Use ras_carla-msg:shift_x instead.")
  (shift_x m))

(cl:ensure-generic-function 'shift_y-val :lambda-list '(m))
(cl:defmethod shift_y-val ((m <RasObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ras_carla-msg:shift_y-val is deprecated.  Use ras_carla-msg:shift_y instead.")
  (shift_y m))

(cl:ensure-generic-function 'is_front-val :lambda-list '(m))
(cl:defmethod is_front-val ((m <RasObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ras_carla-msg:is_front-val is deprecated.  Use ras_carla-msg:is_front instead.")
  (is_front m))

(cl:ensure-generic-function 'importance-val :lambda-list '(m))
(cl:defmethod importance-val ((m <RasObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ras_carla-msg:importance-val is deprecated.  Use ras_carla-msg:importance instead.")
  (importance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RasObject>) ostream)
  "Serializes a message object of type '<RasObject>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shift_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shift_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_front) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'importance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RasObject>) istream)
  "Deserializes a message object of type '<RasObject>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shift_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shift_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'is_front) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'importance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RasObject>)))
  "Returns string type for a message object of type '<RasObject>"
  "ras_carla/RasObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RasObject)))
  "Returns string type for a message object of type 'RasObject"
  "ras_carla/RasObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RasObject>)))
  "Returns md5sum for a message object of type '<RasObject>"
  "6890397792f930487f9106664fc3bee3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RasObject)))
  "Returns md5sum for a message object of type 'RasObject"
  "6890397792f930487f9106664fc3bee3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RasObject>)))
  "Returns full string definition for message of type '<RasObject>"
  (cl:format cl:nil "derived_object_msgs/Object object~%float32 distance~%float32 shift_x~%float32 shift_y~%bool is_front~%float32 importance~%~%================================================================================~%MSG: derived_object_msgs/Object~%# This represents a detected or tracked object with reference coordinate frame and timestamp.~%~%Header header~%~%# The id of the object (presumably from the detecting sensor).~%uint32 id~%~%# A Detected object is one which has been seen in at least one scan/frame of a sensor.~%# A Tracked object is one which has been correlated over multiple scans/frames of a sensor.~%# An object which is detected can only be assumed to have valid pose and shape properties.~%# An object which is tracked should also be assumed to have valid twist and accel properties.~%uint8 detection_level~%~%uint8 OBJECT_DETECTED=0~%uint8 OBJECT_TRACKED=1~%~%# A Classified object is one which has been identified as a certain object type.~%# Classified objects should have valid classification, classification_certainty, and classification_age properties.~%bool object_classified~%~%# The detected position and orientation of the object.~%geometry_msgs/Pose pose~%~%# The detected linear and angular velocities of the object.~%geometry_msgs/Twist twist~%~%# The detected linear and angular accelerations of the object.~%geometry_msgs/Accel accel~%~%# (OPTIONAL) The polygon defining the detection points at the outer edges of the object.~%geometry_msgs/Polygon polygon~%~%# A shape conforming to the outer bounding edges of the object.~%shape_msgs/SolidPrimitive shape~%~%# The type of classification given to this object.~%uint8 classification~%~%uint8 CLASSIFICATION_UNKNOWN=0~%uint8 CLASSIFICATION_UNKNOWN_SMALL=1~%uint8 CLASSIFICATION_UNKNOWN_MEDIUM=2~%uint8 CLASSIFICATION_UNKNOWN_BIG=3~%uint8 CLASSIFICATION_PEDESTRIAN=4~%uint8 CLASSIFICATION_BIKE=5~%uint8 CLASSIFICATION_CAR=6~%uint8 CLASSIFICATION_TRUCK=7~%uint8 CLASSIFICATION_MOTORCYCLE=8~%uint8 CLASSIFICATION_OTHER_VEHICLE=9~%uint8 CLASSIFICATION_BARRIER=10~%uint8 CLASSIFICATION_SIGN=11~%~%# The certainty of the classification from the originating sensor.~%# Higher value indicates greater certainty (MAX=255).~%# It is recommended that a native sensor value be scaled to 0-255 for interoperability.~%uint8 classification_certainty~%~%# The number of scans/frames from the sensor that this object has been classified as the current classification.~%uint32 classification_age~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: shape_msgs/SolidPrimitive~%# Define box, sphere, cylinder, cone ~%# All shapes are defined to have their bounding boxes centered around 0,0,0.~%~%uint8 BOX=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 CONE=4~%~%# The type of the shape~%uint8 type~%~%~%# The dimensions of the shape~%float64[] dimensions~%~%# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array~%~%# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding~%# sides of the box.~%uint8 BOX_X=0~%uint8 BOX_Y=1~%uint8 BOX_Z=2~%~%~%# For the SPHERE type, only one component is used, and it gives the radius of~%# the sphere.~%uint8 SPHERE_RADIUS=0~%~%~%# For the CYLINDER and CONE types, the center line is oriented along~%# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component~%# of dimensions gives the height of the cylinder (cone).  The~%# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the~%# radius of the base of the cylinder (cone).  Cone and cylinder~%# primitives are defined to be circular. The tip of the cone is~%# pointing up, along +Z axis.~%~%uint8 CYLINDER_HEIGHT=0~%uint8 CYLINDER_RADIUS=1~%~%uint8 CONE_HEIGHT=0~%uint8 CONE_RADIUS=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RasObject)))
  "Returns full string definition for message of type 'RasObject"
  (cl:format cl:nil "derived_object_msgs/Object object~%float32 distance~%float32 shift_x~%float32 shift_y~%bool is_front~%float32 importance~%~%================================================================================~%MSG: derived_object_msgs/Object~%# This represents a detected or tracked object with reference coordinate frame and timestamp.~%~%Header header~%~%# The id of the object (presumably from the detecting sensor).~%uint32 id~%~%# A Detected object is one which has been seen in at least one scan/frame of a sensor.~%# A Tracked object is one which has been correlated over multiple scans/frames of a sensor.~%# An object which is detected can only be assumed to have valid pose and shape properties.~%# An object which is tracked should also be assumed to have valid twist and accel properties.~%uint8 detection_level~%~%uint8 OBJECT_DETECTED=0~%uint8 OBJECT_TRACKED=1~%~%# A Classified object is one which has been identified as a certain object type.~%# Classified objects should have valid classification, classification_certainty, and classification_age properties.~%bool object_classified~%~%# The detected position and orientation of the object.~%geometry_msgs/Pose pose~%~%# The detected linear and angular velocities of the object.~%geometry_msgs/Twist twist~%~%# The detected linear and angular accelerations of the object.~%geometry_msgs/Accel accel~%~%# (OPTIONAL) The polygon defining the detection points at the outer edges of the object.~%geometry_msgs/Polygon polygon~%~%# A shape conforming to the outer bounding edges of the object.~%shape_msgs/SolidPrimitive shape~%~%# The type of classification given to this object.~%uint8 classification~%~%uint8 CLASSIFICATION_UNKNOWN=0~%uint8 CLASSIFICATION_UNKNOWN_SMALL=1~%uint8 CLASSIFICATION_UNKNOWN_MEDIUM=2~%uint8 CLASSIFICATION_UNKNOWN_BIG=3~%uint8 CLASSIFICATION_PEDESTRIAN=4~%uint8 CLASSIFICATION_BIKE=5~%uint8 CLASSIFICATION_CAR=6~%uint8 CLASSIFICATION_TRUCK=7~%uint8 CLASSIFICATION_MOTORCYCLE=8~%uint8 CLASSIFICATION_OTHER_VEHICLE=9~%uint8 CLASSIFICATION_BARRIER=10~%uint8 CLASSIFICATION_SIGN=11~%~%# The certainty of the classification from the originating sensor.~%# Higher value indicates greater certainty (MAX=255).~%# It is recommended that a native sensor value be scaled to 0-255 for interoperability.~%uint8 classification_certainty~%~%# The number of scans/frames from the sensor that this object has been classified as the current classification.~%uint32 classification_age~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: shape_msgs/SolidPrimitive~%# Define box, sphere, cylinder, cone ~%# All shapes are defined to have their bounding boxes centered around 0,0,0.~%~%uint8 BOX=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 CONE=4~%~%# The type of the shape~%uint8 type~%~%~%# The dimensions of the shape~%float64[] dimensions~%~%# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array~%~%# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding~%# sides of the box.~%uint8 BOX_X=0~%uint8 BOX_Y=1~%uint8 BOX_Z=2~%~%~%# For the SPHERE type, only one component is used, and it gives the radius of~%# the sphere.~%uint8 SPHERE_RADIUS=0~%~%~%# For the CYLINDER and CONE types, the center line is oriented along~%# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component~%# of dimensions gives the height of the cylinder (cone).  The~%# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the~%# radius of the base of the cylinder (cone).  Cone and cylinder~%# primitives are defined to be circular. The tip of the cone is~%# pointing up, along +Z axis.~%~%uint8 CYLINDER_HEIGHT=0~%uint8 CYLINDER_RADIUS=1~%~%uint8 CONE_HEIGHT=0~%uint8 CONE_RADIUS=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RasObject>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     4
     4
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RasObject>))
  "Converts a ROS message object to a list"
  (cl:list 'RasObject
    (cl:cons ':object (object msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':shift_x (shift_x msg))
    (cl:cons ':shift_y (shift_y msg))
    (cl:cons ':is_front (is_front msg))
    (cl:cons ':importance (importance msg))
))
