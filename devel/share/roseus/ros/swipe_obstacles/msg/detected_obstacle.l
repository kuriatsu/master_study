;; Auto-generated. Do not edit!


(when (boundp 'swipe_obstacles::detected_obstacle)
  (if (not (find-package "SWIPE_OBSTACLES"))
    (make-package "SWIPE_OBSTACLES"))
  (shadow 'detected_obstacle (find-package "SWIPE_OBSTACLES")))
(unless (find-package "SWIPE_OBSTACLES::DETECTED_OBSTACLE")
  (make-package "SWIPE_OBSTACLES::DETECTED_OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude detected_obstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass swipe_obstacles::detected_obstacle
  :super ros::object
  :slots (_header _id _managed_id _label _score _pose _shift_x _shift_y _round _detected_time ))

(defmethod swipe_obstacles::detected_obstacle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:managed_id __managed_id) 0)
    ((:label __label) "")
    ((:score __score) 0.0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:shift_x __shift_x) 0.0)
    ((:shift_y __shift_y) 0.0)
    ((:round __round) 0)
    ((:detected_time __detected_time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _managed_id (round __managed_id))
   (setq _label (string __label))
   (setq _score (float __score))
   (setq _pose __pose)
   (setq _shift_x (float __shift_x))
   (setq _shift_y (float __shift_y))
   (setq _round (round __round))
   (setq _detected_time __detected_time)
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
  (:managed_id
   (&optional __managed_id)
   (if __managed_id (setq _managed_id __managed_id)) _managed_id)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:shift_x
   (&optional __shift_x)
   (if __shift_x (setq _shift_x __shift_x)) _shift_x)
  (:shift_y
   (&optional __shift_y)
   (if __shift_y (setq _shift_y __shift_y)) _shift_y)
  (:round
   (&optional __round)
   (if __round (setq _round __round)) _round)
  (:detected_time
   (&optional __detected_time)
   (if __detected_time (setq _detected_time __detected_time)) _detected_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _id
    4
    ;; uint32 _managed_id
    4
    ;; string _label
    4 (length _label)
    ;; float32 _score
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; float32 _shift_x
    4
    ;; float32 _shift_y
    4
    ;; uint32 _round
    4
    ;; time _detected_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _id
       (write-long _id s)
     ;; uint32 _managed_id
       (write-long _managed_id s)
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _score
       (sys::poke _score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; float32 _shift_x
       (sys::poke _shift_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _shift_y
       (sys::poke _shift_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _round
       (write-long _round s)
     ;; time _detected_time
       (write-long (send _detected_time :sec) s) (write-long (send _detected_time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _managed_id
     (setq _managed_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _score
     (setq _score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float32 _shift_x
     (setq _shift_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _shift_y
     (setq _shift_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _round
     (setq _round (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _detected_time
     (send _detected_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _detected_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get swipe_obstacles::detected_obstacle :md5sum-) "9978e5bb58ee01f8e45fc2d8376b759a")
(setf (get swipe_obstacles::detected_obstacle :datatype-) "swipe_obstacles/detected_obstacle")
(setf (get swipe_obstacles::detected_obstacle :definition-)
      "std_msgs/Header header

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

")



(provide :swipe_obstacles/detected_obstacle "9978e5bb58ee01f8e45fc2d8376b759a")


