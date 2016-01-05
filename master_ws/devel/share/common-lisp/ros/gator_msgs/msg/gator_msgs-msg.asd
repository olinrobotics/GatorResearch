
(cl:in-package :asdf)

(defsystem "gator_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DriveState" :depends-on ("_package_DriveState"))
    (:file "_package_DriveState" :depends-on ("_package"))
  ))