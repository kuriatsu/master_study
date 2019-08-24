
(cl:in-package :asdf)

(defsystem "data_logger-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "swipe_obstacles_log" :depends-on ("_package_swipe_obstacles_log"))
    (:file "_package_swipe_obstacles_log" :depends-on ("_package"))
  ))