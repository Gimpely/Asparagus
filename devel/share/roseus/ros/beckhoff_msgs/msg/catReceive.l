;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::catReceive)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'catReceive (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::CATRECEIVE")
  (make-package "BECKHOFF_MSGS::CATRECEIVE"))

(in-package "ROS")
;;//! \htmlinclude catReceive.msg.html


(defclass beckhoff_msgs::catReceive
  :super ros::object
  :slots (_v _Velocity _AngularVel _X_poz _Y_poz _Th_poz _Th_pozPi ))

(defmethod beckhoff_msgs::catReceive
  (:init
   (&key
    ((:v __v) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:Velocity __Velocity) 0.0)
    ((:AngularVel __AngularVel) 0.0)
    ((:X_poz __X_poz) 0.0)
    ((:Y_poz __Y_poz) 0.0)
    ((:Th_poz __Th_poz) 0.0)
    ((:Th_pozPi __Th_pozPi) 0.0)
    )
   (send-super :init)
   (setq _v __v)
   (setq _Velocity (float __Velocity))
   (setq _AngularVel (float __AngularVel))
   (setq _X_poz (float __X_poz))
   (setq _Y_poz (float __Y_poz))
   (setq _Th_poz (float __Th_poz))
   (setq _Th_pozPi (float __Th_pozPi))
   self)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:Velocity
   (&optional __Velocity)
   (if __Velocity (setq _Velocity __Velocity)) _Velocity)
  (:AngularVel
   (&optional __AngularVel)
   (if __AngularVel (setq _AngularVel __AngularVel)) _AngularVel)
  (:X_poz
   (&optional __X_poz)
   (if __X_poz (setq _X_poz __X_poz)) _X_poz)
  (:Y_poz
   (&optional __Y_poz)
   (if __Y_poz (setq _Y_poz __Y_poz)) _Y_poz)
  (:Th_poz
   (&optional __Th_poz)
   (if __Th_poz (setq _Th_poz __Th_poz)) _Th_poz)
  (:Th_pozPi
   (&optional __Th_pozPi)
   (if __Th_pozPi (setq _Th_pozPi __Th_pozPi)) _Th_pozPi)
  (:serialization-length
   ()
   (+
    ;; float64[2] _v
    (* 8    2)
    ;; float64 _Velocity
    8
    ;; float64 _AngularVel
    8
    ;; float64 _X_poz
    8
    ;; float64 _Y_poz
    8
    ;; float64 _Th_poz
    8
    ;; float64 _Th_pozPi
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[2] _v
     (dotimes (i 2)
       (sys::poke (elt _v i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _Velocity
       (sys::poke _Velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _AngularVel
       (sys::poke _AngularVel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _X_poz
       (sys::poke _X_poz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y_poz
       (sys::poke _Y_poz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Th_poz
       (sys::poke _Th_poz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Th_pozPi
       (sys::poke _Th_pozPi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[2] _v
   (dotimes (i (length _v))
     (setf (elt _v i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _Velocity
     (setq _Velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _AngularVel
     (setq _AngularVel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _X_poz
     (setq _X_poz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y_poz
     (setq _Y_poz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Th_poz
     (setq _Th_poz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Th_pozPi
     (setq _Th_pozPi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get beckhoff_msgs::catReceive :md5sum-) "1e32c3fba4c7a01b6dd0b032a6c34aeb")
(setf (get beckhoff_msgs::catReceive :datatype-) "beckhoff_msgs/catReceive")
(setf (get beckhoff_msgs::catReceive :definition-)
      "float64[2] v
float64 Velocity
float64 AngularVel
float64 X_poz
float64 Y_poz
float64 Th_poz
float64 Th_pozPi
")



(provide :beckhoff_msgs/catReceive "1e32c3fba4c7a01b6dd0b032a6c34aeb")


