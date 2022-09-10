; Auto-generated. Do not edit!


(cl:in-package carTop-msg)


;//! \htmlinclude carTop.msg.html

(cl:defclass <carTop> (roslisp-msg-protocol:ros-message)
  ((back_wheel_speed
    :reader back_wheel_speed
    :initarg :back_wheel_speed
    :type cl:float
    :initform 0.0)
   (turn_angle
    :reader turn_angle
    :initarg :turn_angle
    :type cl:float
    :initform 0.0)
   (battery_level
    :reader battery_level
    :initarg :battery_level
    :type cl:integer
    :initform 0)
   (error_flag
    :reader error_flag
    :initarg :error_flag
    :type cl:integer
    :initform 0))
)

(cl:defclass carTop (<carTop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <carTop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'carTop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name carTop-msg:<carTop> is deprecated: use carTop-msg:carTop instead.")))

(cl:ensure-generic-function 'back_wheel_speed-val :lambda-list '(m))
(cl:defmethod back_wheel_speed-val ((m <carTop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carTop-msg:back_wheel_speed-val is deprecated.  Use carTop-msg:back_wheel_speed instead.")
  (back_wheel_speed m))

(cl:ensure-generic-function 'turn_angle-val :lambda-list '(m))
(cl:defmethod turn_angle-val ((m <carTop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carTop-msg:turn_angle-val is deprecated.  Use carTop-msg:turn_angle instead.")
  (turn_angle m))

(cl:ensure-generic-function 'battery_level-val :lambda-list '(m))
(cl:defmethod battery_level-val ((m <carTop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carTop-msg:battery_level-val is deprecated.  Use carTop-msg:battery_level instead.")
  (battery_level m))

(cl:ensure-generic-function 'error_flag-val :lambda-list '(m))
(cl:defmethod error_flag-val ((m <carTop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carTop-msg:error_flag-val is deprecated.  Use carTop-msg:error_flag instead.")
  (error_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <carTop>) ostream)
  "Serializes a message object of type '<carTop>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'back_wheel_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'turn_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'battery_level)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error_flag)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <carTop>) istream)
  "Deserializes a message object of type '<carTop>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'back_wheel_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'turn_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'battery_level)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error_flag)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<carTop>)))
  "Returns string type for a message object of type '<carTop>"
  "carTop/carTop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'carTop)))
  "Returns string type for a message object of type 'carTop"
  "carTop/carTop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<carTop>)))
  "Returns md5sum for a message object of type '<carTop>"
  "2399edfa2a8160084c724032695e45ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'carTop)))
  "Returns md5sum for a message object of type 'carTop"
  "2399edfa2a8160084c724032695e45ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<carTop>)))
  "Returns full string definition for message of type '<carTop>"
  (cl:format cl:nil "float64 back_wheel_speed~%float64 turn_angle~%char battery_level~%char error_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'carTop)))
  "Returns full string definition for message of type 'carTop"
  (cl:format cl:nil "float64 back_wheel_speed~%float64 turn_angle~%char battery_level~%char error_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <carTop>))
  (cl:+ 0
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <carTop>))
  "Converts a ROS message object to a list"
  (cl:list 'carTop
    (cl:cons ':back_wheel_speed (back_wheel_speed msg))
    (cl:cons ':turn_angle (turn_angle msg))
    (cl:cons ':battery_level (battery_level msg))
    (cl:cons ':error_flag (error_flag msg))
))
