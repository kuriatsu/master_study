;; Auto-generated. Do not edit!


(when (boundp 'swipe_obstacles::closest_obstacle)
  (if (not (find-package "SWIPE_OBSTACLES"))
    (make-package "SWIPE_OBSTACLES"))
  (shadow 'closest_obstacle (find-package "SWIPE_OBSTACLES")))
(unless (find-package "SWIPE_OBSTACLES::CLOSEST_OBSTACLE")
  (make-package "SWIPE_OBSTACLES::CLOSEST_OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude closest_obstacle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass swipe_obstacles::closest_obstacle
  :super ros::object
  :slots (_header _id _distance _brief_stop _stop_time ))

(defmethod swipe_obstacles::closest_obstacle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:distance __distance) 0.0)
    ((:brief_stop __brief_stop) 0)
    ((:stop_time __stop_time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _distance (float __distance))
   (setq _brief_stop (round __brief_stop))
   (setq _stop_time (float __stop_time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:brief_stop
   (&optional __brief_stop)
   (if __brief_stop (setq _brief_stop __brief_stop)) _brief_stop)
  (:stop_time
   (&optional __stop_time)
   (if __stop_time (setq _stop_time __stop_time)) _stop_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _id
    4
    ;; float32 _distance
    4
    ;; uint32 _brief_stop
    4
    ;; float32 _stop_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _id
       (write-long _id s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _brief_stop
       (write-long _brief_stop s)
     ;; float32 _stop_time
       (sys::poke _stop_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _brief_stop
     (setq _brief_stop (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _stop_time
     (setq _stop_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get swipe_obstacles::closest_obstacle :md5sum-) "70db3acc59acd012b3791a69840cb05c")
(setf (get swipe_obstacles::closest_obstacle :datatype-) "swipe_obstacles/closest_obstacle")
(setf (get swipe_obstacles::closest_obstacle :definition-)
      "std_msgs/Header header

uint32 id
float32 distance
uint32 brief_stop
float32 stop_time

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

")



(provide :swipe_obstacles/closest_obstacle "70db3acc59acd012b3791a69840cb05c")


