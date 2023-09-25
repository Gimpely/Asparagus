; Auto-generated. Do not edit!


(cl:in-package plc_to_pc_pub_cpp-msg)


;//! \htmlinclude double5.msg.html

(cl:defclass <double5> (roslisp-msg-protocol:ros-message)
  ((alpha
    :reader alpha
    :initarg :alpha
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass double5 (<double5>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <double5>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'double5)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plc_to_pc_pub_cpp-msg:<double5> is deprecated: use plc_to_pc_pub_cpp-msg:double5 instead.")))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <double5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plc_to_pc_pub_cpp-msg:alpha-val is deprecated.  Use plc_to_pc_pub_cpp-msg:alpha instead.")
  (alpha m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <double5>) ostream)
  "Serializes a message object of type '<double5>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'alpha))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <double5>) istream)
  "Deserializes a message object of type '<double5>"
  (cl:setf (cl:slot-value msg 'alpha) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'alpha)))
    (cl:dotimes (i 5)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<double5>)))
  "Returns string type for a message object of type '<double5>"
  "plc_to_pc_pub_cpp/double5")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'double5)))
  "Returns string type for a message object of type 'double5"
  "plc_to_pc_pub_cpp/double5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<double5>)))
  "Returns md5sum for a message object of type '<double5>"
  "dc8dffcd3a49516a92785ed44605ab1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'double5)))
  "Returns md5sum for a message object of type 'double5"
  "dc8dffcd3a49516a92785ed44605ab1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<double5>)))
  "Returns full string definition for message of type '<double5>"
  (cl:format cl:nil "float64[5] alpha~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'double5)))
  "Returns full string definition for message of type 'double5"
  (cl:format cl:nil "float64[5] alpha~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <double5>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'alpha) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <double5>))
  "Converts a ROS message object to a list"
  (cl:list 'double5
    (cl:cons ':alpha (alpha msg))
))
