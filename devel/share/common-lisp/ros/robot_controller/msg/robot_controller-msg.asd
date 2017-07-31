
(cl:in-package :asdf)

(defsystem "robot_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SpeedUVW" :depends-on ("_package_SpeedUVW"))
    (:file "_package_SpeedUVW" :depends-on ("_package"))
  ))