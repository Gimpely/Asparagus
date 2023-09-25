; Auto-generated. Do not edit!


(cl:in-package beckhoff_msgs-msg)


;//! \htmlinclude Vector_q5.msg.html

(cl:defclass <Vector_q5> (roslisp-msg-protocol:ros-message)
  ((j0
    :reader j0
    :initarg :j0
    :type cl:float
    :initform 0.0)
   (j1
    :reader j1
    :initarg :j1
    :type cl:float
    :initform 0.0)
   (j2
    :reader j2
    :initarg :j2
    :type cl:float
    :initform 0.0)
   (j3
    :reader j3
    :initarg :j3
    :type cl:float
    :initform 0.0)
   (j4
    :reader j4
    :initarg :j4
    :type cl:float
    :initform 0.0))
)

(cl:defclass Vector_q5 (<Vector_q5>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vector_q5>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vector_q5)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beckhoff_msgs-msg:<Vector_q5> is deprecated: use beckhoff_msgs-msg:Vector_q5 instead.")))

(cl:ensure-generic-function 'j0-val :lambda-list '(m))
(cl:defmethod j0-val ((m <Vector_q5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:j0-val is deprecated.  Use beckhoff_msgs-msg:j0 instead.")
  (j0 m))

(cl:ensure-generic-function 'j1-val :lambda-list '(m))
(cl:defmethod j1-val ((m <Vector_q5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:j1-val is deprecated.  Use beckhoff_msgs-msg:j1 instead.")
  (j1 m))

(cl:ensure-generic-function 'j2-val :lambda-list '(m))
(cl:defmethod j2-val ((m <Vector_q5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:j2-val is deprecated.  Use beckhoff_msgs-msg:j2 instead.")
  (j2 m))

(cl:ensure-generic-function 'j3-val :lambda-list '(m))
(cl:defmethod j3-val ((m <Vector_q5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:j3-val is deprecated.  Use beckhoff_msgs-msg:j3 instead.")
  (j3 m))

(cl:ensure-generic-function 'j4-val :lambda-list '(m))
(cl:defmethod j4-val ((m <Vector_q5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:j4-val is deprecated.  Use beckhoff_msgs-msg:j4 instead.")
  (j4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vector_q5>) ostream)
  "Serializes a message object of type '<Vector_q5>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vector_q5>) istream)
  "Deserializes a message object of type '<Vector_q5>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j4) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vector_q5>)))
  "Returns string type for a message object of type '<Vector_q5>"
  "beckhoff_msgs/Vector_q5")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vector_q5)))
  "Returns string type for a message object of type 'Vector_q5"
  "beckhoff_msgs/Vector_q5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vector_q5>)))
  "Returns md5sum for a message object of type '<Vector_q5>"
  "1fa159e5fd623899f47577ffd89d6a4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vector_q5)))
  "Returns md5sum for a message object of type 'Vector_q5"
  "1fa159e5fd623899f47577ffd89d6a4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vector_q5>)))
  "Returns full string definition for message of type '<Vector_q5>"
  (cl:format cl:nil "float32 j0~%float32 j1~%float32 j2~%float32 j3~%float32 j4~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vector_q5)))
  "Returns full string definition for message of type 'Vector_q5"
  (cl:format cl:nil "float32 j0~%float32 j1~%float32 j2~%float32 j3~%float32 j4~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vector_q5>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vector_q5>))
  "Converts a ROS message object to a list"
  (cl:list 'Vector_q5
    (cl:cons ':j0 (j0 msg))
    (cl:cons ':j1 (j1 msg))
    (cl:cons ':j2 (j2 msg))
    (cl:cons ':j3 (j3 msg))
    (cl:cons ':j4 (j4 msg))
))
