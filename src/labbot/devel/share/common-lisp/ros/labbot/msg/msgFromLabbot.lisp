; Auto-generated. Do not edit!


(cl:in-package labbot-msg)


;//! \htmlinclude msgFromLabbot.msg.html

(cl:defclass <msgFromLabbot> (roslisp-msg-protocol:ros-message)
  ((motorRightInput
    :reader motorRightInput
    :initarg :motorRightInput
    :type cl:float
    :initform 0.0)
   (motorRightSetpoint
    :reader motorRightSetpoint
    :initarg :motorRightSetpoint
    :type cl:float
    :initform 0.0)
   (motorRightOutput
    :reader motorRightOutput
    :initarg :motorRightOutput
    :type cl:float
    :initform 0.0)
   (motorLeftInput
    :reader motorLeftInput
    :initarg :motorLeftInput
    :type cl:float
    :initform 0.0)
   (motorLeftSetpoint
    :reader motorLeftSetpoint
    :initarg :motorLeftSetpoint
    :type cl:float
    :initform 0.0)
   (motorLefttOutput
    :reader motorLefttOutput
    :initarg :motorLefttOutput
    :type cl:float
    :initform 0.0))
)

(cl:defclass msgFromLabbot (<msgFromLabbot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgFromLabbot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgFromLabbot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name labbot-msg:<msgFromLabbot> is deprecated: use labbot-msg:msgFromLabbot instead.")))

(cl:ensure-generic-function 'motorRightInput-val :lambda-list '(m))
(cl:defmethod motorRightInput-val ((m <msgFromLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorRightInput-val is deprecated.  Use labbot-msg:motorRightInput instead.")
  (motorRightInput m))

(cl:ensure-generic-function 'motorRightSetpoint-val :lambda-list '(m))
(cl:defmethod motorRightSetpoint-val ((m <msgFromLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorRightSetpoint-val is deprecated.  Use labbot-msg:motorRightSetpoint instead.")
  (motorRightSetpoint m))

(cl:ensure-generic-function 'motorRightOutput-val :lambda-list '(m))
(cl:defmethod motorRightOutput-val ((m <msgFromLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorRightOutput-val is deprecated.  Use labbot-msg:motorRightOutput instead.")
  (motorRightOutput m))

(cl:ensure-generic-function 'motorLeftInput-val :lambda-list '(m))
(cl:defmethod motorLeftInput-val ((m <msgFromLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorLeftInput-val is deprecated.  Use labbot-msg:motorLeftInput instead.")
  (motorLeftInput m))

(cl:ensure-generic-function 'motorLeftSetpoint-val :lambda-list '(m))
(cl:defmethod motorLeftSetpoint-val ((m <msgFromLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorLeftSetpoint-val is deprecated.  Use labbot-msg:motorLeftSetpoint instead.")
  (motorLeftSetpoint m))

(cl:ensure-generic-function 'motorLefttOutput-val :lambda-list '(m))
(cl:defmethod motorLefttOutput-val ((m <msgFromLabbot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader labbot-msg:motorLefttOutput-val is deprecated.  Use labbot-msg:motorLefttOutput instead.")
  (motorLefttOutput m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgFromLabbot>) ostream)
  "Serializes a message object of type '<msgFromLabbot>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorRightInput))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorRightSetpoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorRightOutput))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorLeftInput))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorLeftSetpoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorLefttOutput))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgFromLabbot>) istream)
  "Deserializes a message object of type '<msgFromLabbot>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorRightInput) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorRightSetpoint) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorRightOutput) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorLeftInput) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorLeftSetpoint) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorLefttOutput) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgFromLabbot>)))
  "Returns string type for a message object of type '<msgFromLabbot>"
  "labbot/msgFromLabbot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgFromLabbot)))
  "Returns string type for a message object of type 'msgFromLabbot"
  "labbot/msgFromLabbot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgFromLabbot>)))
  "Returns md5sum for a message object of type '<msgFromLabbot>"
  "4bf0f6e2fd2b507f5c1d51ceec5b93b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgFromLabbot)))
  "Returns md5sum for a message object of type 'msgFromLabbot"
  "4bf0f6e2fd2b507f5c1d51ceec5b93b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgFromLabbot>)))
  "Returns full string definition for message of type '<msgFromLabbot>"
  (cl:format cl:nil "# right motor parameters~%float32 motorRightInput~%float32 motorRightSetpoint~%float32 motorRightOutput~%# left motor parameters~%float32 motorLeftInput~%float32 motorLeftSetpoint~%float32 motorLefttOutput~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgFromLabbot)))
  "Returns full string definition for message of type 'msgFromLabbot"
  (cl:format cl:nil "# right motor parameters~%float32 motorRightInput~%float32 motorRightSetpoint~%float32 motorRightOutput~%# left motor parameters~%float32 motorLeftInput~%float32 motorLeftSetpoint~%float32 motorLefttOutput~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgFromLabbot>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgFromLabbot>))
  "Converts a ROS message object to a list"
  (cl:list 'msgFromLabbot
    (cl:cons ':motorRightInput (motorRightInput msg))
    (cl:cons ':motorRightSetpoint (motorRightSetpoint msg))
    (cl:cons ':motorRightOutput (motorRightOutput msg))
    (cl:cons ':motorLeftInput (motorLeftInput msg))
    (cl:cons ':motorLeftSetpoint (motorLeftSetpoint msg))
    (cl:cons ':motorLefttOutput (motorLefttOutput msg))
))
