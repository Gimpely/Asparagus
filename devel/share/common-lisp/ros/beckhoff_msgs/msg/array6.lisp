; Auto-generated. Do not edit!


(cl:in-package beckhoff_msgs-msg)


;//! \htmlinclude array6.msg.html

(cl:defclass <array6> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass array6 (<array6>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <array6>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'array6)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beckhoff_msgs-msg:<array6> is deprecated: use beckhoff_msgs-msg:array6 instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <array6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:data-val is deprecated.  Use beckhoff_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <array6>) ostream)
  "Serializes a message object of type '<array6>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <array6>) istream)
  "Deserializes a message object of type '<array6>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<array6>)))
  "Returns string type for a message object of type '<array6>"
  "beckhoff_msgs/array6")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'array6)))
  "Returns string type for a message object of type 'array6"
  "beckhoff_msgs/array6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<array6>)))
  "Returns md5sum for a message object of type '<array6>"
  "402f543b13bcc71e911bc511f3982ac2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'array6)))
  "Returns md5sum for a message object of type 'array6"
  "402f543b13bcc71e911bc511f3982ac2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<array6>)))
  "Returns full string definition for message of type '<array6>"
  (cl:format cl:nil "float64[6] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'array6)))
  "Returns full string definition for message of type 'array6"
  (cl:format cl:nil "float64[6] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <array6>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <array6>))
  "Converts a ROS message object to a list"
  (cl:list 'array6
    (cl:cons ':data (data msg))
))
