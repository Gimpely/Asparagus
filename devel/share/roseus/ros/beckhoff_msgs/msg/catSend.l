;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::catSend)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'catSend (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::CATSEND")
  (make-package "BECKHOFF_MSGS::CATSEND"))

(in-package "ROS")
;;//! \htmlinclude catSend.msg.html


(defclass beckhoff_msgs::catSend
  :super ros::object
  :slots (_VelX _VelRot _ResetPoz ))

(defmethod beckhoff_msgs::catSend
  (:init
   (&key
    ((:VelX __VelX) 0.0)
    ((:VelRot __VelRot) 0.0)
    ((:ResetPoz __ResetPoz) nil)
    )
   (send-super :init)
   (setq _VelX (float __VelX))
   (setq _VelRot (float __VelRot))
   (setq _ResetPoz __ResetPoz)
   self)
  (:VelX
   (&optional __VelX)
   (if __VelX (setq _VelX __VelX)) _VelX)
  (:VelRot
   (&optional __VelRot)
   (if __VelRot (setq _VelRot __VelRot)) _VelRot)
  (:ResetPoz
   (&optional (__ResetPoz :null))
   (if (not (eq __ResetPoz :null)) (setq _ResetPoz __ResetPoz)) _ResetPoz)
  (:serialization-length
   ()
   (+
    ;; float64 _VelX
    8
    ;; float64 _VelRot
    8
    ;; bool _ResetPoz
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _VelX
       (sys::poke _VelX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _VelRot
       (sys::poke _VelRot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _ResetPoz
       (if _ResetPoz (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _VelX
     (setq _VelX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _VelRot
     (setq _VelRot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _ResetPoz
     (setq _ResetPoz (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get beckhoff_msgs::catSend :md5sum-) "3e4b9f26ff50bf503fcd6a14b54ee63d")
(setf (get beckhoff_msgs::catSend :datatype-) "beckhoff_msgs/catSend")
(setf (get beckhoff_msgs::catSend :definition-)
      "float64 VelX
float64 VelRot
bool ResetPoz
")



(provide :beckhoff_msgs/catSend "3e4b9f26ff50bf503fcd6a14b54ee63d")


