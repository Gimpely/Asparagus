;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::CmdRobot)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'CmdRobot (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::CMDROBOT")
  (make-package "BECKHOFF_MSGS::CMDROBOT"))

(in-package "ROS")
;;//! \htmlinclude CmdRobot.msg.html


(defclass beckhoff_msgs::CmdRobot
  :super ros::object
  :slots (_Timestamp _dq _home_gripper _open_gripper _close_gripper ))

(defmethod beckhoff_msgs::CmdRobot
  (:init
   (&key
    ((:Timestamp __Timestamp) (instance ros::time :init))
    ((:dq __dq) (instance beckhoff_msgs::Vector_q5 :init))
    ((:home_gripper __home_gripper) nil)
    ((:open_gripper __open_gripper) nil)
    ((:close_gripper __close_gripper) nil)
    )
   (send-super :init)
   (setq _Timestamp __Timestamp)
   (setq _dq __dq)
   (setq _home_gripper __home_gripper)
   (setq _open_gripper __open_gripper)
   (setq _close_gripper __close_gripper)
   self)
  (:Timestamp
   (&optional __Timestamp)
   (if __Timestamp (setq _Timestamp __Timestamp)) _Timestamp)
  (:dq
   (&rest __dq)
   (if (keywordp (car __dq))
       (send* _dq __dq)
     (progn
       (if __dq (setq _dq (car __dq)))
       _dq)))
  (:home_gripper
   (&optional (__home_gripper :null))
   (if (not (eq __home_gripper :null)) (setq _home_gripper __home_gripper)) _home_gripper)
  (:open_gripper
   (&optional (__open_gripper :null))
   (if (not (eq __open_gripper :null)) (setq _open_gripper __open_gripper)) _open_gripper)
  (:close_gripper
   (&optional (__close_gripper :null))
   (if (not (eq __close_gripper :null)) (setq _close_gripper __close_gripper)) _close_gripper)
  (:serialization-length
   ()
   (+
    ;; time _Timestamp
    8
    ;; beckhoff_msgs/Vector_q5 _dq
    (send _dq :serialization-length)
    ;; bool _home_gripper
    1
    ;; bool _open_gripper
    1
    ;; bool _close_gripper
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _Timestamp
       (write-long (send _Timestamp :sec) s) (write-long (send _Timestamp :nsec) s)
     ;; beckhoff_msgs/Vector_q5 _dq
       (send _dq :serialize s)
     ;; bool _home_gripper
       (if _home_gripper (write-byte -1 s) (write-byte 0 s))
     ;; bool _open_gripper
       (if _open_gripper (write-byte -1 s) (write-byte 0 s))
     ;; bool _close_gripper
       (if _close_gripper (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _Timestamp
     (send _Timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _Timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; beckhoff_msgs/Vector_q5 _dq
     (send _dq :deserialize buf ptr-) (incf ptr- (send _dq :serialization-length))
   ;; bool _home_gripper
     (setq _home_gripper (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _open_gripper
     (setq _open_gripper (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _close_gripper
     (setq _close_gripper (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get beckhoff_msgs::CmdRobot :md5sum-) "998ab08fd18737e8efdb2d91fee4e00b")
(setf (get beckhoff_msgs::CmdRobot :datatype-) "beckhoff_msgs/CmdRobot")
(setf (get beckhoff_msgs::CmdRobot :definition-)
      "time Timestamp
Vector_q5 dq
bool home_gripper
bool open_gripper
bool close_gripper
================================================================================
MSG: beckhoff_msgs/Vector_q5
float32 j0
float32 j1
float32 j2
float32 j3
float32 j4
")



(provide :beckhoff_msgs/CmdRobot "998ab08fd18737e8efdb2d91fee4e00b")


