; Auto-generated. Do not edit!


(cl:in-package swipe_obstacles-msg)


;//! \htmlinclude closest_obstacle.msg.html

(cl:defclass <closest_obstacle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (brief_stop
    :reader brief_stop
    :initarg :brief_stop
    :type cl:integer
    :initform 0)
   (stop_time
    :reader stop_time
    :initarg :stop_time
    :type cl:float
    :initform 0.0)
   (stop_distance
    :reader stop_distance
    :initarg :stop_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass closest_obstacle (<closest_obstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <closest_obstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'closest_obstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swipe_obstacles-msg:<closest_obstacle> is deprecated: use swipe_obstacles-msg:closest_obstacle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <closest_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:header-val is deprecated.  Use swipe_obstacles-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <closest_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:id-val is deprecated.  Use swipe_obstacles-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <closest_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:distance-val is deprecated.  Use swipe_obstacles-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'brief_stop-val :lambda-list '(m))
(cl:defmethod brief_stop-val ((m <closest_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:brief_stop-val is deprecated.  Use swipe_obstacles-msg:brief_stop instead.")
  (brief_stop m))

(cl:ensure-generic-function 'stop_time-val :lambda-list '(m))
(cl:defmethod stop_time-val ((m <closest_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:stop_time-val is deprecated.  Use swipe_obstacles-msg:stop_time instead.")
  (stop_time m))

(cl:ensure-generic-function 'stop_distance-val :lambda-list '(m))
(cl:defmethod stop_distance-val ((m <closest_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:stop_distance-val is deprecated.  Use swipe_obstacles-msg:stop_distance instead.")
  (stop_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <closest_obstacle>) ostream)
  "Serializes a message object of type '<closest_obstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brief_stop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brief_stop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'brief_stop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'brief_stop)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stop_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stop_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <closest_obstacle>) istream)
  "Deserializes a message object of type '<closest_obstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brief_stop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brief_stop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'brief_stop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'brief_stop)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stop_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stop_distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<closest_obstacle>)))
  "Returns string type for a message object of type '<closest_obstacle>"
  "swipe_obstacles/closest_obstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'closest_obstacle)))
  "Returns string type for a message object of type 'closest_obstacle"
  "swipe_obstacles/closest_obstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<closest_obstacle>)))
  "Returns md5sum for a message object of type '<closest_obstacle>"
  "f82492c5a8a267aeb81808dbb76fd96b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'closest_obstacle)))
  "Returns md5sum for a message object of type 'closest_obstacle"
  "f82492c5a8a267aeb81808dbb76fd96b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<closest_obstacle>)))
  "Returns full string definition for message of type '<closest_obstacle>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 id~%float32 distance~%uint32 brief_stop~%float32 stop_time~%float32 stop_distance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'closest_obstacle)))
  "Returns full string definition for message of type 'closest_obstacle"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 id~%float32 distance~%uint32 brief_stop~%float32 stop_time~%float32 stop_distance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <closest_obstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <closest_obstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'closest_obstacle
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':brief_stop (brief_stop msg))
    (cl:cons ':stop_time (stop_time msg))
    (cl:cons ':stop_distance (stop_distance msg))
))
