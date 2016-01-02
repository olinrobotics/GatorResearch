
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :diagnostic_msgs-msg
)
  :components ((:file "_package")
    (:file "TestArray" :depends-on ("_package_TestArray"))
    (:file "_package_TestArray" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))