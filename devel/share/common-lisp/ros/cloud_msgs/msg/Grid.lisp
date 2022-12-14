; Auto-generated. Do not edit!


(cl:in-package cloud_msgs-msg)


;//! \htmlinclude Grid.msg.html

(cl:defclass <Grid> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width_step
    :reader width_step
    :initarg :width_step
    :type cl:float
    :initform 0.0)
   (height_step
    :reader height_step
    :initarg :height_step
    :type cl:float
    :initform 0.0)
   (grid_nums
    :reader grid_nums
    :initarg :grid_nums
    :type cl:integer
    :initform 0)
   (grid
    :reader grid
    :initarg :grid
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:fixnum
    :initform 0)
   (pos_vehicle
    :reader pos_vehicle
    :initarg :pos_vehicle
    :type cloud_msgs-msg:PointXYA
    :initform (cl:make-instance 'cloud_msgs-msg:PointXYA)))
)

(cl:defclass Grid (<Grid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Grid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Grid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cloud_msgs-msg:<Grid> is deprecated: use cloud_msgs-msg:Grid instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:timestamp-val is deprecated.  Use cloud_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:width-val is deprecated.  Use cloud_msgs-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:height-val is deprecated.  Use cloud_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width_step-val :lambda-list '(m))
(cl:defmethod width_step-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:width_step-val is deprecated.  Use cloud_msgs-msg:width_step instead.")
  (width_step m))

(cl:ensure-generic-function 'height_step-val :lambda-list '(m))
(cl:defmethod height_step-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:height_step-val is deprecated.  Use cloud_msgs-msg:height_step instead.")
  (height_step m))

(cl:ensure-generic-function 'grid_nums-val :lambda-list '(m))
(cl:defmethod grid_nums-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:grid_nums-val is deprecated.  Use cloud_msgs-msg:grid_nums instead.")
  (grid_nums m))

(cl:ensure-generic-function 'grid-val :lambda-list '(m))
(cl:defmethod grid-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:grid-val is deprecated.  Use cloud_msgs-msg:grid instead.")
  (grid m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:enabled-val is deprecated.  Use cloud_msgs-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'pos_vehicle-val :lambda-list '(m))
(cl:defmethod pos_vehicle-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:pos_vehicle-val is deprecated.  Use cloud_msgs-msg:pos_vehicle instead.")
  (pos_vehicle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Grid>) ostream)
  "Serializes a message object of type '<Grid>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timestamp) (cl:floor (cl:slot-value msg 'timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width_step))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height_step))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'grid_nums)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'grid))
  (cl:let* ((signed (cl:slot-value msg 'enabled)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_vehicle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Grid>) istream)
  "Deserializes a message object of type '<Grid>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width_step) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_step) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'grid_nums) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grid) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grid)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'enabled) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_vehicle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Grid>)))
  "Returns string type for a message object of type '<Grid>"
  "cloud_msgs/Grid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Grid)))
  "Returns string type for a message object of type 'Grid"
  "cloud_msgs/Grid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Grid>)))
  "Returns md5sum for a message object of type '<Grid>"
  "ec789739e2a01936ea531728fbd248c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Grid)))
  "Returns md5sum for a message object of type 'Grid"
  "ec789739e2a01936ea531728fbd248c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Grid>)))
  "Returns full string definition for message of type '<Grid>"
  (cl:format cl:nil "time       timestamp~%int32      width~%int32      height~%float32    width_step~%float32    height_step~%int32      grid_nums~%int8[]     grid~%int8       enabled~%PointXYA   pos_vehicle~%~%================================================================================~%MSG: cloud_msgs/PointXYA~%float64 x~%float64 y~%float64 yaw  # degs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Grid)))
  "Returns full string definition for message of type 'Grid"
  (cl:format cl:nil "time       timestamp~%int32      width~%int32      height~%float32    width_step~%float32    height_step~%int32      grid_nums~%int8[]     grid~%int8       enabled~%PointXYA   pos_vehicle~%~%================================================================================~%MSG: cloud_msgs/PointXYA~%float64 x~%float64 y~%float64 yaw  # degs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Grid>))
  (cl:+ 0
     8
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grid) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_vehicle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Grid>))
  "Converts a ROS message object to a list"
  (cl:list 'Grid
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width_step (width_step msg))
    (cl:cons ':height_step (height_step msg))
    (cl:cons ':grid_nums (grid_nums msg))
    (cl:cons ':grid (grid msg))
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':pos_vehicle (pos_vehicle msg))
))
