; Auto-generated. Do not edit!


(cl:in-package plan_msgs-msg)


;//! \htmlinclude PointSYK.msg.html

(cl:defclass <PointSYK> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (s
    :reader s
    :initarg :s
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
    :initform 0.0))
)

(cl:defclass PointSYK (<PointSYK>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointSYK>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointSYK)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plan_msgs-msg:<PointSYK> is deprecated: use plan_msgs-msg:PointSYK instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <PointSYK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:x-val is deprecated.  Use plan_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <PointSYK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:y-val is deprecated.  Use plan_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <PointSYK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:s-val is deprecated.  Use plan_msgs-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <PointSYK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:yaw-val is deprecated.  Use plan_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'ks-val :lambda-list '(m))
(cl:defmethod ks-val ((m <PointSYK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:ks-val is deprecated.  Use plan_msgs-msg:ks instead.")
  (ks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointSYK>) ostream)
  "Serializes a message object of type '<PointSYK>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's))))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointSYK>) istream)
  "Deserializes a message object of type '<PointSYK>"
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
    (cl:setf (cl:slot-value msg 's) (roslisp-utils:decode-single-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointSYK>)))
  "Returns string type for a message object of type '<PointSYK>"
  "plan_msgs/PointSYK")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointSYK)))
  "Returns string type for a message object of type 'PointSYK"
  "plan_msgs/PointSYK")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointSYK>)))
  "Returns md5sum for a message object of type '<PointSYK>"
  "38aea89911d068b911d894eef5ebbb8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointSYK)))
  "Returns md5sum for a message object of type 'PointSYK"
  "38aea89911d068b911d894eef5ebbb8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointSYK>)))
  "Returns full string definition for message of type '<PointSYK>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 s~%float32 yaw  # rad~%float32 ks~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointSYK)))
  "Returns full string definition for message of type 'PointSYK"
  (cl:format cl:nil "float32 x~%float32 y~%float32 s~%float32 yaw  # rad~%float32 ks~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointSYK>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointSYK>))
  "Converts a ROS message object to a list"
  (cl:list 'PointSYK
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':s (s msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':ks (ks msg))
))
