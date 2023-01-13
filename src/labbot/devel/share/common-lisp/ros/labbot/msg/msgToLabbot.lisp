; Auto-generated. Do not edit!


(cl:in-package labbot-msg)


;//! \htmlinclude msgToLabbot.msg.html

(cl:defclass <msgToLabbot> (roslisp-msg-protocol:ros-message)
  ((motorRightSpeed
    :reader motorRightSpeed
    :initarg :motorRightSpeed
    :type cl:float
    :initform 0.0)
   (motorLeftSpeed
    :reader motorLeftSpeed
    :initarg :motorLeftSpeed
    :type cl:float
    :initform 0.0))
)

(cl:defclass msgToLabbot (<msgToLabbot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgToLabbot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgToLabbot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name labbot-msg:<msgToLabbot> is deprecated: use labbot-msg:msgToLabbot instead.")))

(cl:ensure-generic-function 'motorRightSpeed-val :lambda-list '(m))
(cl:defmethod motorRightSpeed-val ((m <msgToLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorRightSpeed-val is deprecated.  Use labbot-msg:motorRightSpeed instead.")
  (motorRightSpeed m))

(cl:ensure-generic-function 'motorLeftSpeed-val :lambda-list '(m))
(cl:defmethod motorLeftSpeed-val ((m <msgToLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorLeftSpeed-val is deprecated.  Use labbot-msg:motorLeftSpeed instead.")
  (motorLeftSpeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgToLabbot>) ostream)
  "Serializes a message object of type '<msgToLabbot>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorRightSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorLeftSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgToLabbot>) istream)
  "Deserializes a message object of type '<msgToLabbot>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorRightSpeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorLeftSpeed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgToLabbot>)))
  "Returns string type for a message object of type '<msgToLabbot>"
  "labbot/msgToLabbot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgToLabbot)))
  "Returns string type for a message object of type 'msgToLabbot"
  "labbot/msgToLabbot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgToLabbot>)))
  "Returns md5sum for a message object of type '<msgToLabbot>"
  "227e24617faca74e2a267f0d326d13ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgToLabbot)))
  "Returns md5sum for a message object of type 'msgToLabbot"
  "227e24617faca74e2a267f0d326d13ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgToLabbot>)))
  "Returns full string definition for message of type '<msgToLabbot>"
  (cl:format cl:nil "#motor speed (range: 0-80)~%float32 motorRightSpeed~%float32 motorLeftSpeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgToLabbot)))
  "Returns full string definition for message of type 'msgToLabbot"
  (cl:format cl:nil "#motor speed (range: 0-80)~%float32 motorRightSpeed~%float32 motorLeftSpeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgToLabbot>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgToLabbot>))
  "Converts a ROS message object to a list"
  (cl:list 'msgToLabbot
    (cl:cons ':motorRightSpeed (motorRightSpeed msg))
    (cl:cons ':motorLeftSpeed (motorLeftSpeed msg))
))
