;; Auto-generated. Do not edit!


(when (boundp 'data_logger::swipe_obstacles_log)
  (if (not (find-package "DATA_LOGGER"))
    (make-package "DATA_LOGGER"))
  (shadow 'swipe_obstacles_log (find-package "DATA_LOGGER")))
(unless (find-package "DATA_LOGGER::SWIPE_OBSTACLES_LOG")
  (make-package "DATA_LOGGER::SWIPE_OBSTACLES_LOG"))

(in-package "ROS")
;;//! \htmlinclude swipe_obstacles_log.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass data_logger::swipe_obstacles_log
  :super ros::object
  :slots (_header _round _pose _odom _twist _brake _accel _shift _obstacle_id _detected_flag _pedestrian_flag ))

(defmethod data_logger::swipe_obstacles_log
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:round __round) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:odom __odom) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:brake __brake) 0.0)
    ((:accel __accel) 0.0)
    ((:shift __shift) 0.0)
    ((:obstacle_id __obstacle_id) 0)
    ((:detected_flag __detected_flag) 0)
    ((:pedestrian_flag __pedestrian_flag) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _round (round __round))
   (setq _pose __pose)
   (setq _odom __odom)
   (setq _twist __twist)
   (setq _brake (float __brake))
   (setq _accel (float __accel))
   (setq _shift (float __shift))
   (setq _obstacle_id (round __obstacle_id))
   (setq _detected_flag (round __detected_flag))
   (setq _pedestrian_flag (round __pedestrian_flag))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:round
   (&optional __round)
   (if __round (setq _round __round)) _round)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:odom
   (&rest __odom)
   (if (keywordp (car __odom))
       (send* _odom __odom)
     (progn
       (if __odom (setq _odom (car __odom)))
       _odom)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:accel
   (&optional __accel)
   (if __accel (setq _accel __accel)) _accel)
  (:shift
   (&optional __shift)
   (if __shift (setq _shift __shift)) _shift)
  (:obstacle_id
   (&optional __obstacle_id)
   (if __obstacle_id (setq _obstacle_id __obstacle_id)) _obstacle_id)
  (:detected_flag
   (&optional __detected_flag)
   (if __detected_flag (setq _detected_flag __detected_flag)) _detected_flag)
  (:pedestrian_flag
   (&optional __pedestrian_flag)
   (if __pedestrian_flag (setq _pedestrian_flag __pedestrian_flag)) _pedestrian_flag)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _round
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Pose _odom
    (send _odom :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; float32 _brake
    4
    ;; float32 _accel
    4
    ;; float32 _shift
    4
    ;; uint32 _obstacle_id
    4
    ;; uint32 _detected_flag
    4
    ;; uint32 _pedestrian_flag
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _round
       (write-long _round s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Pose _odom
       (send _odom :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; float32 _brake
       (sys::poke _brake (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _accel
       (sys::poke _accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _shift
       (sys::poke _shift (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _obstacle_id
       (write-long _obstacle_id s)
     ;; uint32 _detected_flag
       (write-long _detected_flag s)
     ;; uint32 _pedestrian_flag
       (write-long _pedestrian_flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _round
     (setq _round (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Pose _odom
     (send _odom :deserialize buf ptr-) (incf ptr- (send _odom :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; float32 _brake
     (setq _brake (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _accel
     (setq _accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _shift
     (setq _shift (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _obstacle_id
     (setq _obstacle_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _detected_flag
     (setq _detected_flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pedestrian_flag
     (setq _pedestrian_flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get data_logger::swipe_obstacles_log :md5sum-) "9986bce81c59a09d6f4ebd3feb2a8ca7")
(setf (get data_logger::swipe_obstacles_log :datatype-) "data_logger/swipe_obstacles_log")
(setf (get data_logger::swipe_obstacles_log :definition-)
      "std_msgs/Header header

uint32 round
geometry_msgs/Pose pose
geometry_msgs/Pose odom
geometry_msgs/Twist twist
float32 brake
float32 accel
float32 shift
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
")



(provide :data_logger/swipe_obstacles_log "9986bce81c59a09d6f4ebd3feb2a8ca7")


