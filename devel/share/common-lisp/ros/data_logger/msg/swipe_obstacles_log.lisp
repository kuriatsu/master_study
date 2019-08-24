; Auto-generated. Do not edit!


(cl:in-package data_logger-msg)


;//! \htmlinclude swipe_obstacles_log.msg.html

(cl:defclass <swipe_obstacles_log> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (round
    :reader round
    :initarg :round
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (odom
    :reader odom
    :initarg :odom
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (brake
    :reader brake
    :initarg :brake
    :type cl:float
    :initform 0.0)
   (accel
    :reader accel
    :initarg :accel
    :type cl:float
    :initform 0.0)
   (shift
    :reader shift
    :initarg :shift
    :type cl:float
    :initform 0.0)
   (obstacle_id
    :reader obstacle_id
    :initarg :obstacle_id
    :type cl:integer
    :initform 0)
   (detected_flag
    :reader detected_flag
    :initarg :detected_flag
    :type cl:integer
    :initform 0)
   (pedestrian_flag
    :reader pedestrian_flag
    :initarg :pedestrian_flag
    :type cl:integer
    :initform 0))
)

(cl:defclass swipe_obstacles_log (<swipe_obstacles_log>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <swipe_obstacles_log>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'swipe_obstacles_log)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_logger-msg:<swipe_obstacles_log> is deprecated: use data_logger-msg:swipe_obstacles_log instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:header-val is deprecated.  Use data_logger-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'round-val :lambda-list '(m))
(cl:defmethod round-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:round-val is deprecated.  Use data_logger-msg:round instead.")
  (round m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:pose-val is deprecated.  Use data_logger-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'odom-val :lambda-list '(m))
(cl:defmethod odom-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:odom-val is deprecated.  Use data_logger-msg:odom instead.")
  (odom m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:twist-val is deprecated.  Use data_logger-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:brake-val is deprecated.  Use data_logger-msg:brake instead.")
  (brake m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:accel-val is deprecated.  Use data_logger-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'shift-val :lambda-list '(m))
(cl:defmethod shift-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:shift-val is deprecated.  Use data_logger-msg:shift instead.")
  (shift m))

(cl:ensure-generic-function 'obstacle_id-val :lambda-list '(m))
(cl:defmethod obstacle_id-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:obstacle_id-val is deprecated.  Use data_logger-msg:obstacle_id instead.")
  (obstacle_id m))

(cl:ensure-generic-function 'detected_flag-val :lambda-list '(m))
(cl:defmethod detected_flag-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:detected_flag-val is deprecated.  Use data_logger-msg:detected_flag instead.")
  (detected_flag m))

(cl:ensure-generic-function 'pedestrian_flag-val :lambda-list '(m))
(cl:defmethod pedestrian_flag-val ((m <swipe_obstacles_log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_logger-msg:pedestrian_flag-val is deprecated.  Use data_logger-msg:pedestrian_flag instead.")
  (pedestrian_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <swipe_obstacles_log>) ostream)
  "Serializes a message object of type '<swipe_obstacles_log>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'round)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'round)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'round)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'round)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'odom) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'brake))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shift))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'obstacle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'obstacle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detected_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detected_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detected_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detected_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pedestrian_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pedestrian_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pedestrian_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pedestrian_flag)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <swipe_obstacles_log>) istream)
  "Deserializes a message object of type '<swipe_obstacles_log>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'round)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'round)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'round)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'round)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'odom) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brake) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shift) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'obstacle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'obstacle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detected_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detected_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detected_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detected_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pedestrian_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pedestrian_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pedestrian_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pedestrian_flag)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<swipe_obstacles_log>)))
  "Returns string type for a message object of type '<swipe_obstacles_log>"
  "data_logger/swipe_obstacles_log")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'swipe_obstacles_log)))
  "Returns string type for a message object of type 'swipe_obstacles_log"
  "data_logger/swipe_obstacles_log")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<swipe_obstacles_log>)))
  "Returns md5sum for a message object of type '<swipe_obstacles_log>"
  "9986bce81c59a09d6f4ebd3feb2a8ca7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'swipe_obstacles_log)))
  "Returns md5sum for a message object of type 'swipe_obstacles_log"
  "9986bce81c59a09d6f4ebd3feb2a8ca7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<swipe_obstacles_log>)))
  "Returns full string definition for message of type '<swipe_obstacles_log>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 round~%geometry_msgs/Pose pose~%geometry_msgs/Pose odom~%geometry_msgs/Twist twist~%float32 brake~%float32 accel~%float32 shift~%uint32 obstacle_id~%uint32 detected_flag~%uint32 pedestrian_flag~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'swipe_obstacles_log)))
  "Returns full string definition for message of type 'swipe_obstacles_log"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 round~%geometry_msgs/Pose pose~%geometry_msgs/Pose odom~%geometry_msgs/Twist twist~%float32 brake~%float32 accel~%float32 shift~%uint32 obstacle_id~%uint32 detected_flag~%uint32 pedestrian_flag~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <swipe_obstacles_log>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'odom))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <swipe_obstacles_log>))
  "Converts a ROS message object to a list"
  (cl:list 'swipe_obstacles_log
    (cl:cons ':header (header msg))
    (cl:cons ':round (round msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':odom (odom msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':brake (brake msg))
    (cl:cons ':accel (accel msg))
    (cl:cons ':shift (shift msg))
    (cl:cons ':obstacle_id (obstacle_id msg))
    (cl:cons ':detected_flag (detected_flag msg))
    (cl:cons ':pedestrian_flag (pedestrian_flag msg))
))
