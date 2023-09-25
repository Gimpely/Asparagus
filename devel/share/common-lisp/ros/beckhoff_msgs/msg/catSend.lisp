; Auto-generated. Do not edit!


(cl:in-package beckhoff_msgs-msg)


;//! \htmlinclude catSend.msg.html

(cl:defclass <catSend> (roslisp-msg-protocol:ros-message)
  ((VelX
    :reader VelX
    :initarg :VelX
    :type cl:float
    :initform 0.0)
   (VelRot
    :reader VelRot
    :initarg :VelRot
    :type cl:float
    :initform 0.0)
   (ResetPoz
    :reader ResetPoz
    :initarg :ResetPoz
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass catSend (<catSend>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <catSend>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'catSend)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beckhoff_msgs-msg:<catSend> is deprecated: use beckhoff_msgs-msg:catSend instead.")))

(cl:ensure-generic-function 'VelX-val :lambda-list '(m))
(cl:defmethod VelX-val ((m <catSend>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:VelX-val is deprecated.  Use beckhoff_msgs-msg:VelX instead.")
  (VelX m))

(cl:ensure-generic-function 'VelRot-val :lambda-list '(m))
(cl:defmethod VelRot-val ((m <catSend>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:VelRot-val is deprecated.  Use beckhoff_msgs-msg:VelRot instead.")
  (VelRot m))

(cl:ensure-generic-function 'ResetPoz-val :lambda-list '(m))
(cl:defmethod ResetPoz-val ((m <catSend>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:ResetPoz-val is deprecated.  Use beckhoff_msgs-msg:ResetPoz instead.")
  (ResetPoz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <catSend>) ostream)
  "Serializes a message object of type '<catSend>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'VelX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'VelRot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ResetPoz) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <catSend>) istream)
  "Deserializes a message object of type '<catSend>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VelX) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VelRot) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'ResetPoz) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<catSend>)))
  "Returns string type for a message object of type '<catSend>"
  "beckhoff_msgs/catSend")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'catSend)))
  "Returns string type for a message object of type 'catSend"
  "beckhoff_msgs/catSend")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<catSend>)))
  "Returns md5sum for a message object of type '<catSend>"
  "3e4b9f26ff50bf503fcd6a14b54ee63d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'catSend)))
  "Returns md5sum for a message object of type 'catSend"
  "3e4b9f26ff50bf503fcd6a14b54ee63d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<catSend>)))
  "Returns full string definition for message of type '<catSend>"
  (cl:format cl:nil "float64 VelX~%float64 VelRot~%bool ResetPoz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'catSend)))
  "Returns full string definition for message of type 'catSend"
  (cl:format cl:nil "float64 VelX~%float64 VelRot~%bool ResetPoz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <catSend>))
  (cl:+ 0
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <catSend>))
  "Converts a ROS message object to a list"
  (cl:list 'catSend
    (cl:cons ':VelX (VelX msg))
    (cl:cons ':VelRot (VelRot msg))
    (cl:cons ':ResetPoz (ResetPoz msg))
))
