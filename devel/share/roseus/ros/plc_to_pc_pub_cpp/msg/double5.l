;; Auto-generated. Do not edit!


(when (boundp 'plc_to_pc_pub_cpp::double5)
  (if (not (find-package "PLC_TO_PC_PUB_CPP"))
    (make-package "PLC_TO_PC_PUB_CPP"))
  (shadow 'double5 (find-package "PLC_TO_PC_PUB_CPP")))
(unless (find-package "PLC_TO_PC_PUB_CPP::DOUBLE5")
  (make-package "PLC_TO_PC_PUB_CPP::DOUBLE5"))

(in-package "ROS")
;;//! \htmlinclude double5.msg.html


(defclass plc_to_pc_pub_cpp::double5
  :super ros::object
  :slots (_alpha ))

(defmethod plc_to_pc_pub_cpp::double5
  (:init
   (&key
    ((:alpha __alpha) (make-array 5 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _alpha __alpha)
   self)
  (:alpha
   (&optional __alpha)
   (if __alpha (setq _alpha __alpha)) _alpha)
  (:serialization-length
   ()
   (+
    ;; float64[5] _alpha
    (* 8    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[5] _alpha
     (dotimes (i 5)
       (sys::poke (elt _alpha i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[5] _alpha
   (dotimes (i (length _alpha))
     (setf (elt _alpha i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get plc_to_pc_pub_cpp::double5 :md5sum-) "dc8dffcd3a49516a92785ed44605ab1e")
(setf (get plc_to_pc_pub_cpp::double5 :datatype-) "plc_to_pc_pub_cpp/double5")
(setf (get plc_to_pc_pub_cpp::double5 :definition-)
      "float64[5] alpha


")



(provide :plc_to_pc_pub_cpp/double5 "dc8dffcd3a49516a92785ed44605ab1e")


