; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude TestArray.msg.html

(cl:defclass <TestArray> (roslisp-msg-protocol:ros-message)
  ((testarray
    :reader testarray
    :initarg :testarray
    :type (cl:vector diagnostic_msgs-msg:KeyValue)
   :initform (cl:make-array 0 :element-type 'diagnostic_msgs-msg:KeyValue :initial-element (cl:make-instance 'diagnostic_msgs-msg:KeyValue))))
)

(cl:defclass TestArray (<TestArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TestArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TestArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<TestArray> is deprecated: use beginner_tutorials-msg:TestArray instead.")))

(cl:ensure-generic-function 'testarray-val :lambda-list '(m))
(cl:defmethod testarray-val ((m <TestArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:testarray-val is deprecated.  Use beginner_tutorials-msg:testarray instead.")
  (testarray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TestArray>) ostream)
  "Serializes a message object of type '<TestArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'testarray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'testarray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TestArray>) istream)
  "Deserializes a message object of type '<TestArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'testarray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'testarray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'diagnostic_msgs-msg:KeyValue))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TestArray>)))
  "Returns string type for a message object of type '<TestArray>"
  "beginner_tutorials/TestArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TestArray)))
  "Returns string type for a message object of type 'TestArray"
  "beginner_tutorials/TestArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TestArray>)))
  "Returns md5sum for a message object of type '<TestArray>"
  "51149c98320939ae25f418c95ad7617b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TestArray)))
  "Returns md5sum for a message object of type 'TestArray"
  "51149c98320939ae25f418c95ad7617b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TestArray>)))
  "Returns full string definition for message of type '<TestArray>"
  (cl:format cl:nil "diagnostic_msgs/KeyValue[] testarray~%================================================================================~%MSG: diagnostic_msgs/KeyValue~%string key # what to label this value when viewing~%string value # a value to track over time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TestArray)))
  "Returns full string definition for message of type 'TestArray"
  (cl:format cl:nil "diagnostic_msgs/KeyValue[] testarray~%================================================================================~%MSG: diagnostic_msgs/KeyValue~%string key # what to label this value when viewing~%string value # a value to track over time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TestArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'testarray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TestArray>))
  "Converts a ROS message object to a list"
  (cl:list 'TestArray
    (cl:cons ':testarray (testarray msg))
))
