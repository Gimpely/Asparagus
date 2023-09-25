;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::JointStateRobot)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'JointStateRobot (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::JOINTSTATEROBOT")
  (make-package "BECKHOFF_MSGS::JOINTSTATEROBOT"))

(in-package "ROS")
;;//! \htmlinclude JointStateRobot.msg.html


(defclass beckhoff_msgs::JointStateRobot
  :super ros::object
  :slots (_Timestamp _qq _dq ))

(defmethod beckhoff_msgs::JointStateRobot
  (:init
   (&key
    ((:Timestamp __Timestamp) (instance ros::time :init))
    ((:qq __qq) (instance beckhoff_msgs::Vector_q5 :init))
    ((:dq __dq) (instance beckhoff_msgs::Vector_q5 :init))
    )
   (send-super :init)
   (setq _Timestamp __Timestamp)
   (setq _qq __qq)
   (setq _dq __dq)
   self)
  (:Timestamp
   (&optional __Timestamp)
   (if __Timestamp (setq _Timestamp __Timestamp)) _Timestamp)
  (:qq
   (&rest __qq)
   (if (keywordp (car __qq))
       (send* _qq __qq)
     (progn
       (if __qq (setq _qq (car __qq)))
       _qq)))
  (:dq
   (&rest __dq)
   (if (keywordp (car __dq))
       (send* _dq __dq)
     (progn
       (if __dq (setq _dq (car __dq)))
       _dq)))
  (:serialization-length
   ()
   (+
    ;; time _Timestamp
    8
    ;; beckhoff_msgs/Vector_q5 _qq
    (send _qq :serialization-length)
    ;; beckhoff_msgs/Vector_q5 _dq
    (send _dq :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _Timestamp
       (write-long (send _Timestamp :sec) s) (write-long (send _Timestamp :nsec) s)
     ;; beckhoff_msgs/Vector_q5 _qq
       (send _qq :serialize s)
     ;; beckhoff_msgs/Vector_q5 _dq
       (send _dq :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _Timestamp
     (send _Timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _Timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; beckhoff_msgs/Vector_q5 _qq
     (send _qq :deserialize buf ptr-) (incf ptr- (send _qq :serialization-length))
   ;; beckhoff_msgs/Vector_q5 _dq
     (send _dq :deserialize buf ptr-) (incf ptr- (send _dq :serialization-length))
   ;;
   self)
  )

(setf (get beckhoff_msgs::JointStateRobot :md5sum-) "edf7b25e91309ae0441c692ca3db83b0")
(setf (get beckhoff_msgs::JointStateRobot :datatype-) "beckhoff_msgs/JointStateRobot")
(setf (get beckhoff_msgs::JointStateRobot :definition-)
      "time Timestamp
Vector_q5 qq
Vector_q5 dq
================================================================================
MSG: beckhoff_msgs/Vector_q5
float32 j0
float32 j1
float32 j2
float32 j3
float32 j4
")



(provide :beckhoff_msgs/JointStateRobot "edf7b25e91309ae0441c692ca3db83b0")


