; Auto-generated. Do not edit!


(cl:in-package beckhoff_msgs-msg)


;//! \htmlinclude JointStateRobot.msg.html

(cl:defclass <JointStateRobot> (roslisp-msg-protocol:ros-message)
  ((Timestamp
    :reader Timestamp
    :initarg :Timestamp
    :type cl:real
    :initform 0)
   (qq
    :reader qq
    :initarg :qq
    :type beckhoff_msgs-msg:Vector_q5
    :initform (cl:make-instance 'beckhoff_msgs-msg:Vector_q5))
   (dq
    :reader dq
    :initarg :dq
    :type beckhoff_msgs-msg:Vector_q5
    :initform (cl:make-instance 'beckhoff_msgs-msg:Vector_q5)))
)

(cl:defclass JointStateRobot (<JointStateRobot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointStateRobot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointStateRobot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beckhoff_msgs-msg:<JointStateRobot> is deprecated: use beckhoff_msgs-msg:JointStateRobot instead.")))

(cl:ensure-generic-function 'Timestamp-val :lambda-list '(m))
(cl:defmethod Timestamp-val ((m <JointStateRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:Timestamp-val is deprecated.  Use beckhoff_msgs-msg:Timestamp instead.")
  (Timestamp m))

(cl:ensure-generic-function 'qq-val :lambda-list '(m))
(cl:defmethod qq-val ((m <JointStateRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:qq-val is deprecated.  Use beckhoff_msgs-msg:qq instead.")
  (qq m))

(cl:ensure-generic-function 'dq-val :lambda-list '(m))
(cl:defmethod dq-val ((m <JointStateRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:dq-val is deprecated.  Use beckhoff_msgs-msg:dq instead.")
  (dq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointStateRobot>) ostream)
  "Serializes a message object of type '<JointStateRobot>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'Timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'Timestamp) (cl:floor (cl:slot-value msg 'Timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'qq) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dq) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointStateRobot>) istream)
  "Deserializes a message object of type '<JointStateRobot>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'qq) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dq) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointStateRobot>)))
  "Returns string type for a message object of type '<JointStateRobot>"
  "beckhoff_msgs/JointStateRobot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointStateRobot)))
  "Returns string type for a message object of type 'JointStateRobot"
  "beckhoff_msgs/JointStateRobot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointStateRobot>)))
  "Returns md5sum for a message object of type '<JointStateRobot>"
  "edf7b25e91309ae0441c692ca3db83b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointStateRobot)))
  "Returns md5sum for a message object of type 'JointStateRobot"
  "edf7b25e91309ae0441c692ca3db83b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointStateRobot>)))
  "Returns full string definition for message of type '<JointStateRobot>"
  (cl:format cl:nil "time Timestamp~%Vector_q5 qq~%Vector_q5 dq~%================================================================================~%MSG: beckhoff_msgs/Vector_q5~%float32 j0~%float32 j1~%float32 j2~%float32 j3~%float32 j4~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointStateRobot)))
  "Returns full string definition for message of type 'JointStateRobot"
  (cl:format cl:nil "time Timestamp~%Vector_q5 qq~%Vector_q5 dq~%================================================================================~%MSG: beckhoff_msgs/Vector_q5~%float32 j0~%float32 j1~%float32 j2~%float32 j3~%float32 j4~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointStateRobot>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'qq))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dq))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointStateRobot>))
  "Converts a ROS message object to a list"
  (cl:list 'JointStateRobot
    (cl:cons ':Timestamp (Timestamp msg))
    (cl:cons ':qq (qq msg))
    (cl:cons ':dq (dq msg))
))
