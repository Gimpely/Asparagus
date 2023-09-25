;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::array6)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'array6 (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::ARRAY6")
  (make-package "BECKHOFF_MSGS::ARRAY6"))

(in-package "ROS")
;;//! \htmlinclude array6.msg.html


(defclass beckhoff_msgs::array6
  :super ros::object
  :slots (_data ))

(defmethod beckhoff_msgs::array6
  (:init
   (&key
    ((:data __data) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64[6] _data
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _data
     (dotimes (i 6)
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get beckhoff_msgs::array6 :md5sum-) "402f543b13bcc71e911bc511f3982ac2")
(setf (get beckhoff_msgs::array6 :datatype-) "beckhoff_msgs/array6")
(setf (get beckhoff_msgs::array6 :definition-)
      "float64[6] data
")



(provide :beckhoff_msgs/array6 "402f543b13bcc71e911bc511f3982ac2")


