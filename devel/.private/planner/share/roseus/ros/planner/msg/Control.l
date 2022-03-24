;; Auto-generated. Do not edit!


(when (boundp 'planner::Control)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'Control (find-package "PLANNER")))
(unless (find-package "PLANNER::CONTROL")
  (make-package "PLANNER::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html


(defclass planner::Control
  :super ros::object
  :slots (_omega _a ))

(defmethod planner::Control
  (:init
   (&key
    ((:omega __omega) 0.0)
    ((:a __a) 0.0)
    )
   (send-super :init)
   (setq _omega (float __omega))
   (setq _a (float __a))
   self)
  (:omega
   (&optional __omega)
   (if __omega (setq _omega __omega)) _omega)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; float64 _omega
    8
    ;; float64 _a
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _omega
       (sys::poke _omega (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _omega
     (setq _omega (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get planner::Control :md5sum-) "c838c8a33ea16d85216ae4e8104522a9")
(setf (get planner::Control :datatype-) "planner/Control")
(setf (get planner::Control :definition-)
      "float64 omega
float64 a
")



(provide :planner/Control "c838c8a33ea16d85216ae4e8104522a9")


