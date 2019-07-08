;; Auto-generated. Do not edit!


(when (boundp 'swipe_obstacles::detected_obstacle_array)
  (if (not (find-package "SWIPE_OBSTACLES"))
    (make-package "SWIPE_OBSTACLES"))
  (shadow 'detected_obstacle_array (find-package "SWIPE_OBSTACLES")))
(unless (find-package "SWIPE_OBSTACLES::DETECTED_OBSTACLE_ARRAY")
  (make-package "SWIPE_OBSTACLES::DETECTED_OBSTACLE_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude detected_obstacle_array.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass swipe_obstacles::detected_obstacle_array
  :super ros::object
  :slots (_header _obstacles ))

(defmethod swipe_obstacles::detected_obstacle_array
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) (let (r) (dotimes (i 0) (push (instance swipe_obstacles::detected_obstacle :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; swipe_obstacles/detected_obstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; swipe_obstacles/detected_obstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; swipe_obstacles/detected_obstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance swipe_obstacles::detected_obstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get swipe_obstacles::detected_obstacle_array :md5sum-) "491a0dff1749fdf46f11f15d92874d1b")
(setf (get swipe_obstacles::detected_obstacle_array :datatype-) "swipe_obstacles/detected_obstacle_array")
(setf (get swipe_obstacles::detected_obstacle_array :definition-)
      "std_msgs/Header header
detected_obstacle[] obstacles

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
MSG: swipe_obstacles/detected_obstacle
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

")



(provide :swipe_obstacles/detected_obstacle_array "491a0dff1749fdf46f11f15d92874d1b")


