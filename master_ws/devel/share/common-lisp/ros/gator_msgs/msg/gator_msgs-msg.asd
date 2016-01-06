
(cl:in-package :asdf)

(defsystem "gator_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GlobalPose" :depends-on ("_package_GlobalPose"))
    (:file "_package_GlobalPose" :depends-on ("_package"))
    (:file "DriveState" :depends-on ("_package_DriveState"))
    (:file "_package_DriveState" :depends-on ("_package"))
  ))