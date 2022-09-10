; Auto-generated. Do not edit!


(cl:in-package plan_msgs-msg)


;//! \htmlinclude RobotState.msg.html

(cl:defclass <RobotState> (roslisp-msg-protocol:ros-message)
  ((mition_arrived
    :reader mition_arrived
    :initarg :mition_arrived
    :type cl:boolean
    :initform cl:nil)
   (mition_arrive_num
    :reader mition_arrive_num
    :initarg :mition_arrive_num
    :type cl:integer
    :initform 0)
   (Speed
    :reader Speed
    :initarg :Speed
    :type cl:float
    :initform 0.0)
   (Azimuth
    :reader Azimuth
    :initarg :Azimuth
    :type cl:float
    :initform 0.0)
   (Stop
    :reader Stop
    :initarg :Stop
    :type cl:boolean
    :initform cl:nil)
   (loc_fix
    :reader loc_fix
    :initarg :loc_fix
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotState (<RobotState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plan_msgs-msg:<RobotState> is deprecated: use plan_msgs-msg:RobotState instead.")))

(cl:ensure-generic-function 'mition_arrived-val :lambda-list '(m))
(cl:defmethod mition_arrived-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_arrived-val is deprecated.  Use plan_msgs-msg:mition_arrived instead.")
  (mition_arrived m))

(cl:ensure-generic-function 'mition_arrive_num-val :lambda-list '(m))
(cl:defmethod mition_arrive_num-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_arrive_num-val is deprecated.  Use plan_msgs-msg:mition_arrive_num instead.")
  (mition_arrive_num m))

(cl:ensure-generic-function 'Speed-val :lambda-list '(m))
(cl:defmethod Speed-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:Speed-val is deprecated.  Use plan_msgs-msg:Speed instead.")
  (Speed m))

(cl:ensure-generic-function 'Azimuth-val :lambda-list '(m))
(cl:defmethod Azimuth-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:Azimuth-val is deprecated.  Use plan_msgs-msg:Azimuth instead.")
  (Azimuth m))

(cl:ensure-generic-function 'Stop-val :lambda-list '(m))
(cl:defmethod Stop-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:Stop-val is deprecated.  Use plan_msgs-msg:Stop instead.")
  (Stop m))

(cl:ensure-generic-function 'loc_fix-val :lambda-list '(m))
(cl:defmethod loc_fix-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:loc_fix-val is deprecated.  Use plan_msgs-msg:loc_fix instead.")
  (loc_fix m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotState>) ostream)
  "Serializes a message object of type '<RobotState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mition_arrived) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'mition_arrive_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Stop) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'loc_fix)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotState>) istream)
  "Deserializes a message object of type '<RobotState>"
    (cl:setf (cl:slot-value msg 'mition_arrived) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mition_arrive_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Azimuth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'Stop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'loc_fix) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotState>)))
  "Returns string type for a message object of type '<RobotState>"
  "plan_msgs/RobotState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotState)))
  "Returns string type for a message object of type 'RobotState"
  "plan_msgs/RobotState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotState>)))
  "Returns md5sum for a message object of type '<RobotState>"
  "ab96e98e2f0ba56ad199d5f5d6baa6be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotState)))
  "Returns md5sum for a message object of type 'RobotState"
  "ab96e98e2f0ba56ad199d5f5d6baa6be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotState>)))
  "Returns full string definition for message of type '<RobotState>"
  (cl:format cl:nil "#topic: /RobotState~%bool mition_arrived # arrived~%int32 mition_arrive_num # arrived whitch misstion point~%float32 Speed~%float32 Azimuth~%bool  Stop     # pause or stop flag enable~%int32 loc_fix  # Positioning quality 0: miss; 1: fixed; 2: float;~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotState)))
  "Returns full string definition for message of type 'RobotState"
  (cl:format cl:nil "#topic: /RobotState~%bool mition_arrived # arrived~%int32 mition_arrive_num # arrived whitch misstion point~%float32 Speed~%float32 Azimuth~%bool  Stop     # pause or stop flag enable~%int32 loc_fix  # Positioning quality 0: miss; 1: fixed; 2: float;~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotState>))
  (cl:+ 0
     1
     4
     4
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotState>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotState
    (cl:cons ':mition_arrived (mition_arrived msg))
    (cl:cons ':mition_arrive_num (mition_arrive_num msg))
    (cl:cons ':Speed (Speed msg))
    (cl:cons ':Azimuth (Azimuth msg))
    (cl:cons ':Stop (Stop msg))
    (cl:cons ':loc_fix (loc_fix msg))
))
