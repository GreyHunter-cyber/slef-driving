; Auto-generated. Do not edit!


(cl:in-package plan_msgs-msg)


;//! \htmlinclude PointTraj.msg.html

(cl:defclass <PointTraj> (roslisp-msg-protocol:ros-message)
  ((t
    :reader t
    :initarg :t
    :type cl:integer
    :initform 0)
   (d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0)
   (d_d
    :reader d_d
    :initarg :d_d
    :type cl:float
    :initform 0.0)
   (d_dd
    :reader d_dd
    :initarg :d_dd
    :type cl:float
    :initform 0.0)
   (d_ddd
    :reader d_ddd
    :initarg :d_ddd
    :type cl:float
    :initform 0.0)
   (s
    :reader s
    :initarg :s
    :type cl:float
    :initform 0.0)
   (s_d
    :reader s_d
    :initarg :s_d
    :type cl:float
    :initform 0.0)
   (s_dd
    :reader s_dd
    :initarg :s_dd
    :type cl:float
    :initform 0.0)
   (s_ddd
    :reader s_ddd
    :initarg :s_ddd
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (ks
    :reader ks
    :initarg :ks
    :type cl:float
    :initform 0.0)
   (lx
    :reader lx
    :initarg :lx
    :type cl:float
    :initform 0.0)
   (ly
    :reader ly
    :initarg :ly
    :type cl:float
    :initform 0.0)
   (lyaw
    :reader lyaw
    :initarg :lyaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass PointTraj (<PointTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plan_msgs-msg:<PointTraj> is deprecated: use plan_msgs-msg:PointTraj instead.")))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:t-val is deprecated.  Use plan_msgs-msg:t instead.")
  (t m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:d-val is deprecated.  Use plan_msgs-msg:d instead.")
  (d m))

(cl:ensure-generic-function 'd_d-val :lambda-list '(m))
(cl:defmethod d_d-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:d_d-val is deprecated.  Use plan_msgs-msg:d_d instead.")
  (d_d m))

(cl:ensure-generic-function 'd_dd-val :lambda-list '(m))
(cl:defmethod d_dd-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:d_dd-val is deprecated.  Use plan_msgs-msg:d_dd instead.")
  (d_dd m))

(cl:ensure-generic-function 'd_ddd-val :lambda-list '(m))
(cl:defmethod d_ddd-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:d_ddd-val is deprecated.  Use plan_msgs-msg:d_ddd instead.")
  (d_ddd m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:s-val is deprecated.  Use plan_msgs-msg:s instead.")
  (s m))

(cl:ensure-generic-function 's_d-val :lambda-list '(m))
(cl:defmethod s_d-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:s_d-val is deprecated.  Use plan_msgs-msg:s_d instead.")
  (s_d m))

(cl:ensure-generic-function 's_dd-val :lambda-list '(m))
(cl:defmethod s_dd-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:s_dd-val is deprecated.  Use plan_msgs-msg:s_dd instead.")
  (s_dd m))

(cl:ensure-generic-function 's_ddd-val :lambda-list '(m))
(cl:defmethod s_ddd-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:s_ddd-val is deprecated.  Use plan_msgs-msg:s_ddd instead.")
  (s_ddd m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:x-val is deprecated.  Use plan_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:y-val is deprecated.  Use plan_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:yaw-val is deprecated.  Use plan_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'ks-val :lambda-list '(m))
(cl:defmethod ks-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:ks-val is deprecated.  Use plan_msgs-msg:ks instead.")
  (ks m))

(cl:ensure-generic-function 'lx-val :lambda-list '(m))
(cl:defmethod lx-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:lx-val is deprecated.  Use plan_msgs-msg:lx instead.")
  (lx m))

(cl:ensure-generic-function 'ly-val :lambda-list '(m))
(cl:defmethod ly-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:ly-val is deprecated.  Use plan_msgs-msg:ly instead.")
  (ly m))

(cl:ensure-generic-function 'lyaw-val :lambda-list '(m))
(cl:defmethod lyaw-val ((m <PointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:lyaw-val is deprecated.  Use plan_msgs-msg:lyaw instead.")
  (lyaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointTraj>) ostream)
  "Serializes a message object of type '<PointTraj>"
  (cl:let* ((signed (cl:slot-value msg 't)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_dd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_ddd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's_dd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's_ddd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ly))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lyaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointTraj>) istream)
  "Deserializes a message object of type '<PointTraj>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_dd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_ddd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's_dd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's_ddd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ks) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ly) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lyaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointTraj>)))
  "Returns string type for a message object of type '<PointTraj>"
  "plan_msgs/PointTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointTraj)))
  "Returns string type for a message object of type 'PointTraj"
  "plan_msgs/PointTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointTraj>)))
  "Returns md5sum for a message object of type '<PointTraj>"
  "819fa2e7955aef10d3df960cc6b9813c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointTraj)))
  "Returns md5sum for a message object of type 'PointTraj"
  "819fa2e7955aef10d3df960cc6b9813c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointTraj>)))
  "Returns full string definition for message of type '<PointTraj>"
  (cl:format cl:nil "int32 t~%float32 d~%float32 d_d~%float32 d_dd~%float32 d_ddd~%float32 s~%float32 s_d~%float32 s_dd~%float32 s_ddd~%float32 x~%float32 y~%float32 yaw~%float32 ks~%float32 lx~%float32 ly~%float32 lyaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointTraj)))
  "Returns full string definition for message of type 'PointTraj"
  (cl:format cl:nil "int32 t~%float32 d~%float32 d_d~%float32 d_dd~%float32 d_ddd~%float32 s~%float32 s_d~%float32 s_dd~%float32 s_ddd~%float32 x~%float32 y~%float32 yaw~%float32 ks~%float32 lx~%float32 ly~%float32 lyaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointTraj>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'PointTraj
    (cl:cons ':t (t msg))
    (cl:cons ':d (d msg))
    (cl:cons ':d_d (d_d msg))
    (cl:cons ':d_dd (d_dd msg))
    (cl:cons ':d_ddd (d_ddd msg))
    (cl:cons ':s (s msg))
    (cl:cons ':s_d (s_d msg))
    (cl:cons ':s_dd (s_dd msg))
    (cl:cons ':s_ddd (s_ddd msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':ks (ks msg))
    (cl:cons ':lx (lx msg))
    (cl:cons ':ly (ly msg))
    (cl:cons ':lyaw (lyaw msg))
))
