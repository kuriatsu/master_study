
(cl:in-package :asdf)

(defsystem "swipe_obstacles-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "closest_obstacle" :depends-on ("_package_closest_obstacle"))
    (:file "_package_closest_obstacle" :depends-on ("_package"))
    (:file "detected_obstacle" :depends-on ("_package_detected_obstacle"))
    (:file "_package_detected_obstacle" :depends-on ("_package"))
    (:file "detected_obstacle_array" :depends-on ("_package_detected_obstacle_array"))
    (:file "_package_detected_obstacle_array" :depends-on ("_package"))
  ))