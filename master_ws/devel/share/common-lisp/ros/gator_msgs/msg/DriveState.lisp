; Auto-generated. Do not edit!


(cl:in-package gator_msgs-msg)


;//! \htmlinclude DriveState.msg.html

(cl:defclass <DriveState> (roslisp-msg-protocol:ros-message)
  ((steeringangle
    :reader steeringangle
    :initarg :steeringangle
    :type cl:float
    :initform 0.0)
   (wheelvelocity
    :reader wheelvelocity
    :initarg :wheelvelocity
    :type cl:float
    :initform 0.0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DriveState (<DriveState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gator_msgs-msg:<DriveState> is deprecated: use gator_msgs-msg:DriveState instead.")))

(cl:ensure-generic-function 'steeringangle-val :lambda-list '(m))
(cl:defmethod steeringangle-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gator_msgs-msg:steeringangle-val is deprecated.  Use gator_msgs-msg:steeringangle instead.")
  (steeringangle m))

(cl:ensure-generic-function 'wheelvelocity-val :lambda-list '(m))
(cl:defmethod wheelvelocity-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gator_msgs-msg:wheelvelocity-val is deprecated.  Use gator_msgs-msg:wheelvelocity instead.")
  (wheelvelocity m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gator_msgs-msg:brake-val is deprecated.  Use gator_msgs-msg:brake instead.")
  (brake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveState>) ostream)
  "Serializes a message object of type '<DriveState>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steeringangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wheelvelocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brake) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveState>) istream)
  "Deserializes a message object of type '<DriveState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steeringangle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheelvelocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'brake) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveState>)))
  "Returns string type for a message object of type '<DriveState>"
  "gator_msgs/DriveState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveState)))
  "Returns string type for a message object of type 'DriveState"
  "gator_msgs/DriveState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveState>)))
  "Returns md5sum for a message object of type '<DriveState>"
  "e9add9a38f3f6f218376f584f3063c5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveState)))
  "Returns md5sum for a message object of type 'DriveState"
  "e9add9a38f3f6f218376f584f3063c5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveState>)))
  "Returns full string definition for message of type '<DriveState>"
  (cl:format cl:nil "float32 steeringangle~%float32 wheelvelocity~%bool    brake~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveState)))
  "Returns full string definition for message of type 'DriveState"
  (cl:format cl:nil "float32 steeringangle~%float32 wheelvelocity~%bool    brake~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveState>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveState>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveState
    (cl:cons ':steeringangle (steeringangle msg))
    (cl:cons ':wheelvelocity (wheelvelocity msg))
    (cl:cons ':brake (brake msg))
))
