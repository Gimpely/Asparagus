; Auto-generated. Do not edit!


(cl:in-package beckhoff_msgs-msg)


;//! \htmlinclude CmdTracks.msg.html

(cl:defclass <CmdTracks> (roslisp-msg-protocol:ros-message)
  ((stop_tracks
    :reader stop_tracks
    :initarg :stop_tracks
    :type cl:boolean
    :initform cl:nil)
   (start_tracks
    :reader start_tracks
    :initarg :start_tracks
    :type cl:boolean
    :initform cl:nil)
   (linear_vel
    :reader linear_vel
    :initarg :linear_vel
    :type cl:float
    :initform 0.0)
   (rot_vel
    :reader rot_vel
    :initarg :rot_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass CmdTracks (<CmdTracks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CmdTracks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CmdTracks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beckhoff_msgs-msg:<CmdTracks> is deprecated: use beckhoff_msgs-msg:CmdTracks instead.")))

(cl:ensure-generic-function 'stop_tracks-val :lambda-list '(m))
(cl:defmethod stop_tracks-val ((m <CmdTracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:stop_tracks-val is deprecated.  Use beckhoff_msgs-msg:stop_tracks instead.")
  (stop_tracks m))

(cl:ensure-generic-function 'start_tracks-val :lambda-list '(m))
(cl:defmethod start_tracks-val ((m <CmdTracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:start_tracks-val is deprecated.  Use beckhoff_msgs-msg:start_tracks instead.")
  (start_tracks m))

(cl:ensure-generic-function 'linear_vel-val :lambda-list '(m))
(cl:defmethod linear_vel-val ((m <CmdTracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:linear_vel-val is deprecated.  Use beckhoff_msgs-msg:linear_vel instead.")
  (linear_vel m))

(cl:ensure-generic-function 'rot_vel-val :lambda-list '(m))
(cl:defmethod rot_vel-val ((m <CmdTracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beckhoff_msgs-msg:rot_vel-val is deprecated.  Use beckhoff_msgs-msg:rot_vel instead.")
  (rot_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CmdTracks>) ostream)
  "Serializes a message object of type '<CmdTracks>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_tracks) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_tracks) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rot_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CmdTracks>) istream)
  "Deserializes a message object of type '<CmdTracks>"
    (cl:setf (cl:slot-value msg 'stop_tracks) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'start_tracks) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rot_vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CmdTracks>)))
  "Returns string type for a message object of type '<CmdTracks>"
  "beckhoff_msgs/CmdTracks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdTracks)))
  "Returns string type for a message object of type 'CmdTracks"
  "beckhoff_msgs/CmdTracks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CmdTracks>)))
  "Returns md5sum for a message object of type '<CmdTracks>"
  "9944d5598f5dddfb3ecdf391f9c85ed5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CmdTracks)))
  "Returns md5sum for a message object of type 'CmdTracks"
  "9944d5598f5dddfb3ecdf391f9c85ed5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CmdTracks>)))
  "Returns full string definition for message of type '<CmdTracks>"
  (cl:format cl:nil "bool stop_tracks~%bool start_tracks~%float64 linear_vel~%float64 rot_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CmdTracks)))
  "Returns full string definition for message of type 'CmdTracks"
  (cl:format cl:nil "bool stop_tracks~%bool start_tracks~%float64 linear_vel~%float64 rot_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CmdTracks>))
  (cl:+ 0
     1
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CmdTracks>))
  "Converts a ROS message object to a list"
  (cl:list 'CmdTracks
    (cl:cons ':stop_tracks (stop_tracks msg))
    (cl:cons ':start_tracks (start_tracks msg))
    (cl:cons ':linear_vel (linear_vel msg))
    (cl:cons ':rot_vel (rot_vel msg))
))
