; Auto-generated. Do not edit!


(cl:in-package plan_msgs-msg)


;//! \htmlinclude Traj.msg.html

(cl:defclass <Traj> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0)
   (points
    :reader points
    :initarg :points
    :type (cl:vector plan_msgs-msg:PointTraj)
   :initform (cl:make-array 0 :element-type 'plan_msgs-msg:PointTraj :initial-element (cl:make-instance 'plan_msgs-msg:PointTraj))))
)

(cl:defclass Traj (<Traj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Traj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Traj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plan_msgs-msg:<Traj> is deprecated: use plan_msgs-msg:Traj instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <Traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:num-val is deprecated.  Use plan_msgs-msg:num instead.")
  (num m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:points-val is deprecated.  Use plan_msgs-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Traj>) ostream)
  "Serializes a message object of type '<Traj>"
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Traj>) istream)
  "Deserializes a message object of type '<Traj>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'plan_msgs-msg:PointTraj))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Traj>)))
  "Returns string type for a message object of type '<Traj>"
  "plan_msgs/Traj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Traj)))
  "Returns string type for a message object of type 'Traj"
  "plan_msgs/Traj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Traj>)))
  "Returns md5sum for a message object of type '<Traj>"
  "490cb3b1cf624497cbfba91636b0dd01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Traj)))
  "Returns md5sum for a message object of type 'Traj"
  "490cb3b1cf624497cbfba91636b0dd01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Traj>)))
  "Returns full string definition for message of type '<Traj>"
  (cl:format cl:nil "int32     num~%PointTraj[]  points~%~%================================================================================~%MSG: plan_msgs/PointTraj~%int32 t~%float32 d~%float32 d_d~%float32 d_dd~%float32 d_ddd~%float32 s~%float32 s_d~%float32 s_dd~%float32 s_ddd~%float32 x~%float32 y~%float32 yaw~%float32 ks~%float32 lx~%float32 ly~%float32 lyaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Traj)))
  "Returns full string definition for message of type 'Traj"
  (cl:format cl:nil "int32     num~%PointTraj[]  points~%~%================================================================================~%MSG: plan_msgs/PointTraj~%int32 t~%float32 d~%float32 d_d~%float32 d_dd~%float32 d_ddd~%float32 s~%float32 s_d~%float32 s_dd~%float32 s_ddd~%float32 x~%float32 y~%float32 yaw~%float32 ks~%float32 lx~%float32 ly~%float32 lyaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Traj>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Traj>))
  "Converts a ROS message object to a list"
  (cl:list 'Traj
    (cl:cons ':num (num msg))
    (cl:cons ':points (points msg))
))
