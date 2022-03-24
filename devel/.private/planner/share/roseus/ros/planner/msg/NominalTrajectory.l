;; Auto-generated. Do not edit!


(when (boundp 'planner::NominalTrajectory)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'NominalTrajectory (find-package "PLANNER")))
(unless (find-package "PLANNER::NOMINALTRAJECTORY")
  (make-package "PLANNER::NOMINALTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude NominalTrajectory.msg.html


(defclass planner::NominalTrajectory
  :super ros::object
  :slots (_states _controls ))

(defmethod planner::NominalTrajectory
  (:init
   (&key
    ((:states __states) ())
    ((:controls __controls) ())
    )
   (send-super :init)
   (setq _states __states)
   (setq _controls __controls)
   self)
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:controls
   (&rest __controls)
   (if (keywordp (car __controls))
       (send* _controls __controls)
     (progn
       (if __controls (setq _controls (car __controls)))
       _controls)))
  (:serialization-length
   ()
   (+
    ;; planner/State[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ;; planner/Control[] _controls
    (apply #'+ (send-all _controls :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; planner/State[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;; planner/Control[] _controls
     (write-long (length _controls) s)
     (dolist (elem _controls)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; planner/State[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance planner::State :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; planner/Control[] _controls
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controls (let (r) (dotimes (i n) (push (instance planner::Control :init) r)) r))
     (dolist (elem- _controls)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get planner::NominalTrajectory :md5sum-) "8aa3d7b09e7dbafc476534ce12baadb2")
(setf (get planner::NominalTrajectory :datatype-) "planner/NominalTrajectory")
(setf (get planner::NominalTrajectory :definition-)
      "State[] states
Control[] controls
================================================================================
MSG: planner/State
float64 x
float64 y
float64 theta
float64 v
================================================================================
MSG: planner/Control
float64 omega
float64 a
")



(provide :planner/NominalTrajectory "8aa3d7b09e7dbafc476534ce12baadb2")


