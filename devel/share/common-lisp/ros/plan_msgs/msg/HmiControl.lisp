; Auto-generated. Do not edit!


(cl:in-package plan_msgs-msg)


;//! \htmlinclude HmiControl.msg.html

(cl:defclass <HmiControl> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (e_stop
    :reader e_stop
    :initarg :e_stop
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (ang_velo
    :reader ang_velo
    :initarg :ang_velo
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (control_flag
    :reader control_flag
    :initarg :control_flag
    :type cl:integer
    :initform 0)
   (action_flag
    :reader action_flag
    :initarg :action_flag
    :type cl:integer
    :initform 0)
   (mition_num
    :reader mition_num
    :initarg :mition_num
    :type cl:integer
    :initform 0)
   (mition_point_x
    :reader mition_point_x
    :initarg :mition_point_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mition_point_y
    :reader mition_point_y
    :initarg :mition_point_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mition_point_speed
    :reader mition_point_speed
    :initarg :mition_point_speed
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mition_point_a
    :reader mition_point_a
    :initarg :mition_point_a
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (origin_x
    :reader origin_x
    :initarg :origin_x
    :type cl:float
    :initform 0.0)
   (origin_y
    :reader origin_y
    :initarg :origin_y
    :type cl:float
    :initform 0.0)
   (origin_z
    :reader origin_z
    :initarg :origin_z
    :type cl:float
    :initform 0.0)
   (origin_yaw
    :reader origin_yaw
    :initarg :origin_yaw
    :type cl:float
    :initform 0.0)
   (mition_finish
    :reader mition_finish
    :initarg :mition_finish
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HmiControl (<HmiControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HmiControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HmiControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plan_msgs-msg:<HmiControl> is deprecated: use plan_msgs-msg:HmiControl instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:stamp-val is deprecated.  Use plan_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'e_stop-val :lambda-list '(m))
(cl:defmethod e_stop-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:e_stop-val is deprecated.  Use plan_msgs-msg:e_stop instead.")
  (e_stop m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:speed-val is deprecated.  Use plan_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'ang_velo-val :lambda-list '(m))
(cl:defmethod ang_velo-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:ang_velo-val is deprecated.  Use plan_msgs-msg:ang_velo instead.")
  (ang_velo m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:distance-val is deprecated.  Use plan_msgs-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:angle-val is deprecated.  Use plan_msgs-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'control_flag-val :lambda-list '(m))
(cl:defmethod control_flag-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:control_flag-val is deprecated.  Use plan_msgs-msg:control_flag instead.")
  (control_flag m))

(cl:ensure-generic-function 'action_flag-val :lambda-list '(m))
(cl:defmethod action_flag-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:action_flag-val is deprecated.  Use plan_msgs-msg:action_flag instead.")
  (action_flag m))

(cl:ensure-generic-function 'mition_num-val :lambda-list '(m))
(cl:defmethod mition_num-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_num-val is deprecated.  Use plan_msgs-msg:mition_num instead.")
  (mition_num m))

(cl:ensure-generic-function 'mition_point_x-val :lambda-list '(m))
(cl:defmethod mition_point_x-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_point_x-val is deprecated.  Use plan_msgs-msg:mition_point_x instead.")
  (mition_point_x m))

(cl:ensure-generic-function 'mition_point_y-val :lambda-list '(m))
(cl:defmethod mition_point_y-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_point_y-val is deprecated.  Use plan_msgs-msg:mition_point_y instead.")
  (mition_point_y m))

(cl:ensure-generic-function 'mition_point_speed-val :lambda-list '(m))
(cl:defmethod mition_point_speed-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_point_speed-val is deprecated.  Use plan_msgs-msg:mition_point_speed instead.")
  (mition_point_speed m))

(cl:ensure-generic-function 'mition_point_a-val :lambda-list '(m))
(cl:defmethod mition_point_a-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_point_a-val is deprecated.  Use plan_msgs-msg:mition_point_a instead.")
  (mition_point_a m))

(cl:ensure-generic-function 'origin_x-val :lambda-list '(m))
(cl:defmethod origin_x-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:origin_x-val is deprecated.  Use plan_msgs-msg:origin_x instead.")
  (origin_x m))

(cl:ensure-generic-function 'origin_y-val :lambda-list '(m))
(cl:defmethod origin_y-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:origin_y-val is deprecated.  Use plan_msgs-msg:origin_y instead.")
  (origin_y m))

(cl:ensure-generic-function 'origin_z-val :lambda-list '(m))
(cl:defmethod origin_z-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:origin_z-val is deprecated.  Use plan_msgs-msg:origin_z instead.")
  (origin_z m))

(cl:ensure-generic-function 'origin_yaw-val :lambda-list '(m))
(cl:defmethod origin_yaw-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:origin_yaw-val is deprecated.  Use plan_msgs-msg:origin_yaw instead.")
  (origin_yaw m))

(cl:ensure-generic-function 'mition_finish-val :lambda-list '(m))
(cl:defmethod mition_finish-val ((m <HmiControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plan_msgs-msg:mition_finish-val is deprecated.  Use plan_msgs-msg:mition_finish instead.")
  (mition_finish m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HmiControl>) ostream)
  "Serializes a message object of type '<HmiControl>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'e_stop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ang_velo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'control_flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'action_flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mition_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mition_point_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mition_point_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mition_point_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mition_point_y))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mition_point_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mition_point_speed))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mition_point_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mition_point_a))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'origin_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'origin_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'origin_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'origin_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mition_finish) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HmiControl>) istream)
  "Deserializes a message object of type '<HmiControl>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'e_stop) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ang_velo) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_flag) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_flag) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mition_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mition_point_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mition_point_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mition_point_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mition_point_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mition_point_speed) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mition_point_speed)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mition_point_a) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mition_point_a)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'origin_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'origin_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'origin_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'origin_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'mition_finish) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HmiControl>)))
  "Returns string type for a message object of type '<HmiControl>"
  "plan_msgs/HmiControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HmiControl)))
  "Returns string type for a message object of type 'HmiControl"
  "plan_msgs/HmiControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HmiControl>)))
  "Returns md5sum for a message object of type '<HmiControl>"
  "dd2b29065bf0fb426c0ae55225b9903d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HmiControl)))
  "Returns md5sum for a message object of type 'HmiControl"
  "dd2b29065bf0fb426c0ae55225b9903d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HmiControl>)))
  "Returns full string definition for message of type '<HmiControl>"
  (cl:format cl:nil "time    stamp             # timestamp~%int32   e_stop            # emergency stop flag~%float32 speed             # speed in m/s~%float32 ang_velo          # turn angle velocity deg/s~%float32 distance          # run distance in m~%float32 angle             # turn angle in degree(L:+/R:-)~%int32   control_flag      # is start the remote control 0 means free ，1 means pause ,2 means stop ,3 means remote~%int32   action_flag       # 0 means none ,1 means apply mitionPoint ,2 means navigation,3 reset init location~%int32   mition_num        # mitionPoint number~%float32[] mition_point_x  # mitionPoint x~%float32[] mition_point_y  # mitionPoint y~%float32[] mition_point_speed  # mitionPoint speed~%int32[]   mition_point_a  # mitionPoint attribute~%float32 origin_x~%float32 origin_y~%float32 origin_z~%float32 origin_yaw~%bool mition_finish~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HmiControl)))
  "Returns full string definition for message of type 'HmiControl"
  (cl:format cl:nil "time    stamp             # timestamp~%int32   e_stop            # emergency stop flag~%float32 speed             # speed in m/s~%float32 ang_velo          # turn angle velocity deg/s~%float32 distance          # run distance in m~%float32 angle             # turn angle in degree(L:+/R:-)~%int32   control_flag      # is start the remote control 0 means free ，1 means pause ,2 means stop ,3 means remote~%int32   action_flag       # 0 means none ,1 means apply mitionPoint ,2 means navigation,3 reset init location~%int32   mition_num        # mitionPoint number~%float32[] mition_point_x  # mitionPoint x~%float32[] mition_point_y  # mitionPoint y~%float32[] mition_point_speed  # mitionPoint speed~%int32[]   mition_point_a  # mitionPoint attribute~%float32 origin_x~%float32 origin_y~%float32 origin_z~%float32 origin_yaw~%bool mition_finish~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HmiControl>))
  (cl:+ 0
     8
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mition_point_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mition_point_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mition_point_speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mition_point_a) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HmiControl>))
  "Converts a ROS message object to a list"
  (cl:list 'HmiControl
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':e_stop (e_stop msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':ang_velo (ang_velo msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':control_flag (control_flag msg))
    (cl:cons ':action_flag (action_flag msg))
    (cl:cons ':mition_num (mition_num msg))
    (cl:cons ':mition_point_x (mition_point_x msg))
    (cl:cons ':mition_point_y (mition_point_y msg))
    (cl:cons ':mition_point_speed (mition_point_speed msg))
    (cl:cons ':mition_point_a (mition_point_a msg))
    (cl:cons ':origin_x (origin_x msg))
    (cl:cons ':origin_y (origin_y msg))
    (cl:cons ':origin_z (origin_z msg))
    (cl:cons ':origin_yaw (origin_yaw msg))
    (cl:cons ':mition_finish (mition_finish msg))
))
