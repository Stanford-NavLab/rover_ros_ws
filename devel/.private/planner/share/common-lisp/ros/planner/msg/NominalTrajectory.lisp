; Auto-generated. Do not edit!


(cl:in-package planner-msg)


;//! \htmlinclude NominalTrajectory.msg.html

(cl:defclass <NominalTrajectory> (roslisp-msg-protocol:ros-message)
  ((states
    :reader states
    :initarg :states
    :type (cl:vector planner-msg:State)
   :initform (cl:make-array 0 :element-type 'planner-msg:State :initial-element (cl:make-instance 'planner-msg:State)))
   (controls
    :reader controls
    :initarg :controls
    :type (cl:vector planner-msg:Control)
   :initform (cl:make-array 0 :element-type 'planner-msg:Control :initial-element (cl:make-instance 'planner-msg:Control))))
)

(cl:defclass NominalTrajectory (<NominalTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NominalTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NominalTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-msg:<NominalTrajectory> is deprecated: use planner-msg:NominalTrajectory instead.")))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <NominalTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:states-val is deprecated.  Use planner-msg:states instead.")
  (states m))

(cl:ensure-generic-function 'controls-val :lambda-list '(m))
(cl:defmethod controls-val ((m <NominalTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:controls-val is deprecated.  Use planner-msg:controls instead.")
  (controls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NominalTrajectory>) ostream)
  "Serializes a message object of type '<NominalTrajectory>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controls))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controls))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NominalTrajectory>) istream)
  "Deserializes a message object of type '<NominalTrajectory>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'planner-msg:State))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controls) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controls)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'planner-msg:Control))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NominalTrajectory>)))
  "Returns string type for a message object of type '<NominalTrajectory>"
  "planner/NominalTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NominalTrajectory)))
  "Returns string type for a message object of type 'NominalTrajectory"
  "planner/NominalTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NominalTrajectory>)))
  "Returns md5sum for a message object of type '<NominalTrajectory>"
  "8aa3d7b09e7dbafc476534ce12baadb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NominalTrajectory)))
  "Returns md5sum for a message object of type 'NominalTrajectory"
  "8aa3d7b09e7dbafc476534ce12baadb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NominalTrajectory>)))
  "Returns full string definition for message of type '<NominalTrajectory>"
  (cl:format cl:nil "State[] states~%Control[] controls~%================================================================================~%MSG: planner/State~%float64 x~%float64 y~%float64 theta~%float64 v~%================================================================================~%MSG: planner/Control~%float64 omega~%float64 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NominalTrajectory)))
  "Returns full string definition for message of type 'NominalTrajectory"
  (cl:format cl:nil "State[] states~%Control[] controls~%================================================================================~%MSG: planner/State~%float64 x~%float64 y~%float64 theta~%float64 v~%================================================================================~%MSG: planner/Control~%float64 omega~%float64 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NominalTrajectory>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controls) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NominalTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'NominalTrajectory
    (cl:cons ':states (states msg))
    (cl:cons ':controls (controls msg))
))
