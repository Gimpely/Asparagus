;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::dataArray)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'dataArray (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::DATAARRAY")
  (make-package "BECKHOFF_MSGS::DATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude dataArray.msg.html


(defclass beckhoff_msgs::dataArray
  :super ros::object
  :slots (_data ))

(defmethod beckhoff_msgs::dataArray
  (:init
   (&key
    ((:data __data) (make-array 20 :initial-element 0.0 :element-type :float))
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
    ;; float32[20] _data
    (* 4    20)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[20] _data
     (dotimes (i 20)
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[20] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get beckhoff_msgs::dataArray :md5sum-) "863ff43db2ffc1168b53b76ab57ce6a3")
(setf (get beckhoff_msgs::dataArray :datatype-) "beckhoff_msgs/dataArray")
(setf (get beckhoff_msgs::dataArray :definition-)
      "float32[20] data
")



(provide :beckhoff_msgs/dataArray "863ff43db2ffc1168b53b76ab57ce6a3")


