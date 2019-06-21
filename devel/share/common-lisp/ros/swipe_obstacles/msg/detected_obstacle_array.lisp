; Auto-generated. Do not edit!


(cl:in-package swipe_obstacles-msg)


;//! \htmlinclude detected_obstacle_array.msg.html

(cl:defclass <detected_obstacle_array> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector swipe_obstacles-msg:detected_obstacle)
   :initform (cl:make-array 0 :element-type 'swipe_obstacles-msg:detected_obstacle :initial-element (cl:make-instance 'swipe_obstacles-msg:detected_obstacle))))
)

(cl:defclass detected_obstacle_array (<detected_obstacle_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detected_obstacle_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detected_obstacle_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swipe_obstacles-msg:<detected_obstacle_array> is deprecated: use swipe_obstacles-msg:detected_obstacle_array instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <detected_obstacle_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:header-val is deprecated.  Use swipe_obstacles-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <detected_obstacle_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swipe_obstacles-msg:obstacles-val is deprecated.  Use swipe_obstacles-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detected_obstacle_array>) ostream)
  "Serializes a message object of type '<detected_obstacle_array>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detected_obstacle_array>) istream)
  "Deserializes a message object of type '<detected_obstacle_array>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'swipe_obstacles-msg:detected_obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detected_obstacle_array>)))
  "Returns string type for a message object of type '<detected_obstacle_array>"
  "swipe_obstacles/detected_obstacle_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detected_obstacle_array)))
  "Returns string type for a message object of type 'detected_obstacle_array"
  "swipe_obstacles/detected_obstacle_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detected_obstacle_array>)))
  "Returns md5sum for a message object of type '<detected_obstacle_array>"
  "53847f7fcca9cc1d891c94d84db3bd10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detected_obstacle_array)))
  "Returns md5sum for a message object of type 'detected_obstacle_array"
  "53847f7fcca9cc1d891c94d84db3bd10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detected_obstacle_array>)))
  "Returns full string definition for message of type '<detected_obstacle_array>"
  (cl:format cl:nil "std_msgs/Header header~%detected_obstacle[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: swipe_obstacles/detected_obstacle~%std_msgs/Header header~%~%uint32 id~%string label~%float32 score~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detected_obstacle_array)))
  "Returns full string definition for message of type 'detected_obstacle_array"
  (cl:format cl:nil "std_msgs/Header header~%detected_obstacle[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: swipe_obstacles/detected_obstacle~%std_msgs/Header header~%~%uint32 id~%string label~%float32 score~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detected_obstacle_array>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detected_obstacle_array>))
  "Converts a ROS message object to a list"
  (cl:list 'detected_obstacle_array
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
