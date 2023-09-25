;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::Vector_q5)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'Vector_q5 (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::VECTOR_Q5")
  (make-package "BECKHOFF_MSGS::VECTOR_Q5"))

(in-package "ROS")
;;//! \htmlinclude Vector_q5.msg.html


(defclass beckhoff_msgs::Vector_q5
  :super ros::object
  :slots (_j0 _j1 _j2 _j3 _j4 ))

(defmethod beckhoff_msgs::Vector_q5
  (:init
   (&key
    ((:j0 __j0) 0.0)
    ((:j1 __j1) 0.0)
    ((:j2 __j2) 0.0)
    ((:j3 __j3) 0.0)
    ((:j4 __j4) 0.0)
    )
   (send-super :init)
   (setq _j0 (float __j0))
   (setq _j1 (float __j1))
   (setq _j2 (float __j2))
   (setq _j3 (float __j3))
   (setq _j4 (float __j4))
   self)
  (:j0
   (&optional __j0)
   (if __j0 (setq _j0 __j0)) _j0)
  (:j1
   (&optional __j1)
   (if __j1 (setq _j1 __j1)) _j1)
  (:j2
   (&optional __j2)
   (if __j2 (setq _j2 __j2)) _j2)
  (:j3
   (&optional __j3)
   (if __j3 (setq _j3 __j3)) _j3)
  (:j4
   (&optional __j4)
   (if __j4 (setq _j4 __j4)) _j4)
  (:serialization-length
   ()
   (+
    ;; float32 _j0
    4
    ;; float32 _j1
    4
    ;; float32 _j2
    4
    ;; float32 _j3
    4
    ;; float32 _j4
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _j0
       (sys::poke _j0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _j1
       (sys::poke _j1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _j2
       (sys::poke _j2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _j3
       (sys::poke _j3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _j4
       (sys::poke _j4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _j0
     (setq _j0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _j1
     (setq _j1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _j2
     (setq _j2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _j3
     (setq _j3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _j4
     (setq _j4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get beckhoff_msgs::Vector_q5 :md5sum-) "1fa159e5fd623899f47577ffd89d6a4d")
(setf (get beckhoff_msgs::Vector_q5 :datatype-) "beckhoff_msgs/Vector_q5")
(setf (get beckhoff_msgs::Vector_q5 :definition-)
      "float32 j0
float32 j1
float32 j2
float32 j3
float32 j4
")



(provide :beckhoff_msgs/Vector_q5 "1fa159e5fd623899f47577ffd89d6a4d")


