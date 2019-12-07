
(cl:in-package :asdf)

(defsystem "ras_carla-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :derived_object_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RasObject" :depends-on ("_package_RasObject"))
    (:file "_package_RasObject" :depends-on ("_package"))
    (:file "RasObjectArray" :depends-on ("_package_RasObjectArray"))
    (:file "_package_RasObjectArray" :depends-on ("_package"))
  ))