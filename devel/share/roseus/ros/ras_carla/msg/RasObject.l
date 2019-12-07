;; Auto-generated. Do not edit!


(when (boundp 'ras_carla::RasObject)
  (if (not (find-package "RAS_CARLA"))
    (make-package "RAS_CARLA"))
  (shadow 'RasObject (find-package "RAS_CARLA")))
(unless (find-package "RAS_CARLA::RASOBJECT")
  (make-package "RAS_CARLA::RASOBJECT"))

(in-package "ROS")
;;//! \htmlinclude RasObject.msg.html
(if (not (find-package "DERIVED_OBJECT_MSGS"))
  (ros::roseus-add-msgs "derived_object_msgs"))


(defclass ras_carla::RasObject
  :super ros::object
  :slots (_object _distance _shift_x _shift_y _is_front _importance ))

(defmethod ras_carla::RasObject
  (:init
   (&key
    ((:object __object) (instance derived_object_msgs::Object :init))
    ((:distance __distance) 0.0)
    ((:shift_x __shift_x) 0.0)
    ((:shift_y __shift_y) 0.0)
    ((:is_front __is_front) nil)
    ((:importance __importance) 0.0)
    )
   (send-super :init)
   (setq _object __object)
   (setq _distance (float __distance))
   (setq _shift_x (float __shift_x))
   (setq _shift_y (float __shift_y))
   (setq _is_front __is_front)
   (setq _importance (float __importance))
   self)
  (:object
   (&rest __object)
   (if (keywordp (car __object))
       (send* _object __object)
     (progn
       (if __object (setq _object (car __object)))
       _object)))
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:shift_x
   (&optional __shift_x)
   (if __shift_x (setq _shift_x __shift_x)) _shift_x)
  (:shift_y
   (&optional __shift_y)
   (if __shift_y (setq _shift_y __shift_y)) _shift_y)
  (:is_front
   (&optional __is_front)
   (if __is_front (setq _is_front __is_front)) _is_front)
  (:importance
   (&optional __importance)
   (if __importance (setq _importance __importance)) _importance)
  (:serialization-length
   ()
   (+
    ;; derived_object_msgs/Object _object
    (send _object :serialization-length)
    ;; float32 _distance
    4
    ;; float32 _shift_x
    4
    ;; float32 _shift_y
    4
    ;; bool _is_front
    1
    ;; float32 _importance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; derived_object_msgs/Object _object
       (send _object :serialize s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _shift_x
       (sys::poke _shift_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _shift_y
       (sys::poke _shift_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _is_front
       (if _is_front (write-byte -1 s) (write-byte 0 s))
     ;; float32 _importance
       (sys::poke _importance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; derived_object_msgs/Object _object
     (send _object :deserialize buf ptr-) (incf ptr- (send _object :serialization-length))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _shift_x
     (setq _shift_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _shift_y
     (setq _shift_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _is_front
     (setq _is_front (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _importance
     (setq _importance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ras_carla::RasObject :md5sum-) "6890397792f930487f9106664fc3bee3")
(setf (get ras_carla::RasObject :datatype-) "ras_carla/RasObject")
(setf (get ras_carla::RasObject :definition-)
      "derived_object_msgs/Object object
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

")



(provide :ras_carla/RasObject "6890397792f930487f9106664fc3bee3")


