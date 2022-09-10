;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::HmiControl)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'HmiControl (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::HMICONTROL")
  (make-package "PLAN_MSGS::HMICONTROL"))

(in-package "ROS")
;;//! \htmlinclude HmiControl.msg.html


(defclass plan_msgs::HmiControl
  :super ros::object
  :slots (_stamp _e_stop _speed _ang_velo _distance _angle _control_flag _action_flag _mition_num _mition_point_x _mition_point_y _mition_point_speed _mition_point_a _origin_x _origin_y _origin_z _origin_yaw _mition_finish ))

(defmethod plan_msgs::HmiControl
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:e_stop __e_stop) 0)
    ((:speed __speed) 0.0)
    ((:ang_velo __ang_velo) 0.0)
    ((:distance __distance) 0.0)
    ((:angle __angle) 0.0)
    ((:control_flag __control_flag) 0)
    ((:action_flag __action_flag) 0)
    ((:mition_num __mition_num) 0)
    ((:mition_point_x __mition_point_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mition_point_y __mition_point_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mition_point_speed __mition_point_speed) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mition_point_a __mition_point_a) (make-array 0 :initial-element 0 :element-type :integer))
    ((:origin_x __origin_x) 0.0)
    ((:origin_y __origin_y) 0.0)
    ((:origin_z __origin_z) 0.0)
    ((:origin_yaw __origin_yaw) 0.0)
    ((:mition_finish __mition_finish) nil)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _e_stop (round __e_stop))
   (setq _speed (float __speed))
   (setq _ang_velo (float __ang_velo))
   (setq _distance (float __distance))
   (setq _angle (float __angle))
   (setq _control_flag (round __control_flag))
   (setq _action_flag (round __action_flag))
   (setq _mition_num (round __mition_num))
   (setq _mition_point_x __mition_point_x)
   (setq _mition_point_y __mition_point_y)
   (setq _mition_point_speed __mition_point_speed)
   (setq _mition_point_a __mition_point_a)
   (setq _origin_x (float __origin_x))
   (setq _origin_y (float __origin_y))
   (setq _origin_z (float __origin_z))
   (setq _origin_yaw (float __origin_yaw))
   (setq _mition_finish __mition_finish)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:e_stop
   (&optional __e_stop)
   (if __e_stop (setq _e_stop __e_stop)) _e_stop)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:ang_velo
   (&optional __ang_velo)
   (if __ang_velo (setq _ang_velo __ang_velo)) _ang_velo)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:control_flag
   (&optional __control_flag)
   (if __control_flag (setq _control_flag __control_flag)) _control_flag)
  (:action_flag
   (&optional __action_flag)
   (if __action_flag (setq _action_flag __action_flag)) _action_flag)
  (:mition_num
   (&optional __mition_num)
   (if __mition_num (setq _mition_num __mition_num)) _mition_num)
  (:mition_point_x
   (&optional __mition_point_x)
   (if __mition_point_x (setq _mition_point_x __mition_point_x)) _mition_point_x)
  (:mition_point_y
   (&optional __mition_point_y)
   (if __mition_point_y (setq _mition_point_y __mition_point_y)) _mition_point_y)
  (:mition_point_speed
   (&optional __mition_point_speed)
   (if __mition_point_speed (setq _mition_point_speed __mition_point_speed)) _mition_point_speed)
  (:mition_point_a
   (&optional __mition_point_a)
   (if __mition_point_a (setq _mition_point_a __mition_point_a)) _mition_point_a)
  (:origin_x
   (&optional __origin_x)
   (if __origin_x (setq _origin_x __origin_x)) _origin_x)
  (:origin_y
   (&optional __origin_y)
   (if __origin_y (setq _origin_y __origin_y)) _origin_y)
  (:origin_z
   (&optional __origin_z)
   (if __origin_z (setq _origin_z __origin_z)) _origin_z)
  (:origin_yaw
   (&optional __origin_yaw)
   (if __origin_yaw (setq _origin_yaw __origin_yaw)) _origin_yaw)
  (:mition_finish
   (&optional __mition_finish)
   (if __mition_finish (setq _mition_finish __mition_finish)) _mition_finish)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; int32 _e_stop
    4
    ;; float32 _speed
    4
    ;; float32 _ang_velo
    4
    ;; float32 _distance
    4
    ;; float32 _angle
    4
    ;; int32 _control_flag
    4
    ;; int32 _action_flag
    4
    ;; int32 _mition_num
    4
    ;; float32[] _mition_point_x
    (* 4    (length _mition_point_x)) 4
    ;; float32[] _mition_point_y
    (* 4    (length _mition_point_y)) 4
    ;; float32[] _mition_point_speed
    (* 4    (length _mition_point_speed)) 4
    ;; int32[] _mition_point_a
    (* 4    (length _mition_point_a)) 4
    ;; float32 _origin_x
    4
    ;; float32 _origin_y
    4
    ;; float32 _origin_z
    4
    ;; float32 _origin_yaw
    4
    ;; bool _mition_finish
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _e_stop
       (write-long _e_stop s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ang_velo
       (sys::poke _ang_velo (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _control_flag
       (write-long _control_flag s)
     ;; int32 _action_flag
       (write-long _action_flag s)
     ;; int32 _mition_num
       (write-long _mition_num s)
     ;; float32[] _mition_point_x
     (write-long (length _mition_point_x) s)
     (dotimes (i (length _mition_point_x))
       (sys::poke (elt _mition_point_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _mition_point_y
     (write-long (length _mition_point_y) s)
     (dotimes (i (length _mition_point_y))
       (sys::poke (elt _mition_point_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _mition_point_speed
     (write-long (length _mition_point_speed) s)
     (dotimes (i (length _mition_point_speed))
       (sys::poke (elt _mition_point_speed i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _mition_point_a
     (write-long (length _mition_point_a) s)
     (dotimes (i (length _mition_point_a))
       (write-long (elt _mition_point_a i) s)
       )
     ;; float32 _origin_x
       (sys::poke _origin_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _origin_y
       (sys::poke _origin_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _origin_z
       (sys::poke _origin_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _origin_yaw
       (sys::poke _origin_yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _mition_finish
       (if _mition_finish (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _e_stop
     (setq _e_stop (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ang_velo
     (setq _ang_velo (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _control_flag
     (setq _control_flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _action_flag
     (setq _action_flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _mition_num
     (setq _mition_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _mition_point_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mition_point_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _mition_point_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _mition_point_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mition_point_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _mition_point_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _mition_point_speed
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mition_point_speed (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _mition_point_speed i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _mition_point_a
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mition_point_a (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _mition_point_a i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32 _origin_x
     (setq _origin_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _origin_y
     (setq _origin_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _origin_z
     (setq _origin_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _origin_yaw
     (setq _origin_yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _mition_finish
     (setq _mition_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get plan_msgs::HmiControl :md5sum-) "dd2b29065bf0fb426c0ae55225b9903d")
(setf (get plan_msgs::HmiControl :datatype-) "plan_msgs/HmiControl")
(setf (get plan_msgs::HmiControl :definition-)
      "time    stamp             # timestamp
int32   e_stop            # emergency stop flag
float32 speed             # speed in m/s
float32 ang_velo          # turn angle velocity deg/s
float32 distance          # run distance in m
float32 angle             # turn angle in degree(L:+/R:-)
int32   control_flag      # is start the remote control 0 means free ，1 means pause ,2 means stop ,3 means remote
int32   action_flag       # 0 means none ,1 means apply mitionPoint ,2 means navigation,3 reset init location
int32   mition_num        # mitionPoint number
float32[] mition_point_x  # mitionPoint x
float32[] mition_point_y  # mitionPoint y
float32[] mition_point_speed  # mitionPoint speed
int32[]   mition_point_a  # mitionPoint attribute
float32 origin_x
float32 origin_y
float32 origin_z
float32 origin_yaw
bool mition_finish
")



(provide :plan_msgs/HmiControl "dd2b29065bf0fb426c0ae55225b9903d")


