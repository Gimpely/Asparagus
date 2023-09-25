;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::array5)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'array5 (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::ARRAY5")
  (make-package "BECKHOFF_MSGS::ARRAY5"))

(in-package "ROS")
;;//! \htmlinclude array5.msg.html


(defclass beckhoff_msgs::array5
  :super ros::object
  :slots (_data ))

(defmethod beckhoff_msgs::array5
  (:init
   (&key
    ((:data __data) (make-array 5 :initial-element 0.0 :element-type :float))
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
    ;; float64[5] _data
    (* 8    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[5] _data
     (dotimes (i 5)
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[5] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get beckhoff_msgs::array5 :md5sum-) "ebfb6f4de85d3d955b5442f86a122aa7")
(setf (get beckhoff_msgs::array5 :datatype-) "beckhoff_msgs/array5")
(setf (get beckhoff_msgs::array5 :definition-)
      "float64[5] data
")



(provide :beckhoff_msgs/array5 "ebfb6f4de85d3d955b5442f86a122aa7")


