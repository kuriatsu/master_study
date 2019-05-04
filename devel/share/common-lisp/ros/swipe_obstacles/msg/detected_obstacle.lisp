; Auto-generated. Do not edit!


(cl:in-package swipe_obstacles-msg)


;//! \htmlinclude detected_obstacle.msg.html

(cl:defclass <detected_obstacle> (roslisp-msg-protocol:ros-message)
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
   (label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion)))
)

(cl:defclass detected_obstacle (<detected_obstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detected_obstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detected_obstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swipe_obstacles-msg:<detected_obstacle> is deprecated: use swipe_obstacles-msg:detected_obstacle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <detected_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:header-val is deprecated.  Use swipe_obstacles-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <detected_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:id-val is deprecated.  Use swipe_obstacles-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <detected_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:label-val is deprecated.  Use swipe_obstacles-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <detected_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:score-val is deprecated.  Use swipe_obstacles-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <detected_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:position-val is deprecated.  Use swipe_obstacles-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <detected_obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:orientation-val is deprecated.  Use swipe_obstacles-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detected_obstacle>) ostream)
  "Serializes a message object of type '<detected_obstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detected_obstacle>) istream)
  "Deserializes a message object of type '<detected_obstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detected_obstacle>)))
  "Returns string type for a message object of type '<detected_obstacle>"
  "swipe_obstacles/detected_obstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detected_obstacle)))
  "Returns string type for a message object of type 'detected_obstacle"
  "swipe_obstacles/detected_obstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detected_obstacle>)))
  "Returns md5sum for a message object of type '<detected_obstacle>"
  "17028d23d94e8b0c59623a0698afe007")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detected_obstacle)))
  "Returns md5sum for a message object of type 'detected_obstacle"
  "17028d23d94e8b0c59623a0698afe007")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detected_obstacle>)))
  "Returns full string definition for message of type '<detected_obstacle>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 id~%string label~%float32 score~%~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detected_obstacle)))
  "Returns full string definition for message of type 'detected_obstacle"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 id~%string label~%float32 score~%~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detected_obstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:length (cl:slot-value msg 'label))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detected_obstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'detected_obstacle
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':label (label msg))
    (cl:cons ':score (score msg))
    (cl:cons ':position (position msg))
    (cl:cons ':orientation (orientation msg))
))
