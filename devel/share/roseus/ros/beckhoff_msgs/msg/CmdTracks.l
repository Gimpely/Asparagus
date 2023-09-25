;; Auto-generated. Do not edit!


(when (boundp 'beckhoff_msgs::CmdTracks)
  (if (not (find-package "BECKHOFF_MSGS"))
    (make-package "BECKHOFF_MSGS"))
  (shadow 'CmdTracks (find-package "BECKHOFF_MSGS")))
(unless (find-package "BECKHOFF_MSGS::CMDTRACKS")
  (make-package "BECKHOFF_MSGS::CMDTRACKS"))

(in-package "ROS")
;;//! \htmlinclude CmdTracks.msg.html


(defclass beckhoff_msgs::CmdTracks
  :super ros::object
  :slots (_stop_tracks _start_tracks _linear_vel _rot_vel ))

(defmethod beckhoff_msgs::CmdTracks
  (:init
   (&key
    ((:stop_tracks __stop_tracks) nil)
    ((:start_tracks __start_tracks) nil)
    ((:linear_vel __linear_vel) 0.0)
    ((:rot_vel __rot_vel) 0.0)
    )
   (send-super :init)
   (setq _stop_tracks __stop_tracks)
   (setq _start_tracks __start_tracks)
   (setq _linear_vel (float __linear_vel))
   (setq _rot_vel (float __rot_vel))
   self)
  (:stop_tracks
   (&optional (__stop_tracks :null))
   (if (not (eq __stop_tracks :null)) (setq _stop_tracks __stop_tracks)) _stop_tracks)
  (:start_tracks
   (&optional (__start_tracks :null))
   (if (not (eq __start_tracks :null)) (setq _start_tracks __start_tracks)) _start_tracks)
  (:linear_vel
   (&optional __linear_vel)
   (if __linear_vel (setq _linear_vel __linear_vel)) _linear_vel)
  (:rot_vel
   (&optional __rot_vel)
   (if __rot_vel (setq _rot_vel __rot_vel)) _rot_vel)
  (:serialization-length
   ()
   (+
    ;; bool _stop_tracks
    1
    ;; bool _start_tracks
    1
    ;; float64 _linear_vel
    8
    ;; float64 _rot_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _stop_tracks
       (if _stop_tracks (write-byte -1 s) (write-byte 0 s))
     ;; bool _start_tracks
       (if _start_tracks (write-byte -1 s) (write-byte 0 s))
     ;; float64 _linear_vel
       (sys::poke _linear_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rot_vel
       (sys::poke _rot_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _stop_tracks
     (setq _stop_tracks (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _start_tracks
     (setq _start_tracks (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _linear_vel
     (setq _linear_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rot_vel
     (setq _rot_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get beckhoff_msgs::CmdTracks :md5sum-) "9944d5598f5dddfb3ecdf391f9c85ed5")
(setf (get beckhoff_msgs::CmdTracks :datatype-) "beckhoff_msgs/CmdTracks")
(setf (get beckhoff_msgs::CmdTracks :definition-)
      "bool stop_tracks
bool start_tracks
float64 linear_vel
float64 rot_vel
")



(provide :beckhoff_msgs/CmdTracks "9944d5598f5dddfb3ecdf391f9c85ed5")


