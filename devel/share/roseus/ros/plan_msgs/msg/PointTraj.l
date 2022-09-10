;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::PointTraj)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'PointTraj (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::POINTTRAJ")
  (make-package "PLAN_MSGS::POINTTRAJ"))

(in-package "ROS")
;;//! \htmlinclude PointTraj.msg.html


(defclass plan_msgs::PointTraj
  :super ros::object
  :slots (_t _d _d_d _d_dd _d_ddd _s _s_d _s_dd _s_ddd _x _y _yaw _ks _lx _ly _lyaw ))

(defmethod plan_msgs::PointTraj
  (:init
   (&key
    ((:t __t) 0)
    ((:d __d) 0.0)
    ((:d_d __d_d) 0.0)
    ((:d_dd __d_dd) 0.0)
    ((:d_ddd __d_ddd) 0.0)
    ((:s __s) 0.0)
    ((:s_d __s_d) 0.0)
    ((:s_dd __s_dd) 0.0)
    ((:s_ddd __s_ddd) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:ks __ks) 0.0)
    ((:lx __lx) 0.0)
    ((:ly __ly) 0.0)
    ((:lyaw __lyaw) 0.0)
    )
   (send-super :init)
   (setq _t (round __t))
   (setq _d (float __d))
   (setq _d_d (float __d_d))
   (setq _d_dd (float __d_dd))
   (setq _d_ddd (float __d_ddd))
   (setq _s (float __s))
   (setq _s_d (float __s_d))
   (setq _s_dd (float __s_dd))
   (setq _s_ddd (float __s_ddd))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _ks (float __ks))
   (setq _lx (float __lx))
   (setq _ly (float __ly))
   (setq _lyaw (float __lyaw))
   self)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:d_d
   (&optional __d_d)
   (if __d_d (setq _d_d __d_d)) _d_d)
  (:d_dd
   (&optional __d_dd)
   (if __d_dd (setq _d_dd __d_dd)) _d_dd)
  (:d_ddd
   (&optional __d_ddd)
   (if __d_ddd (setq _d_ddd __d_ddd)) _d_ddd)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:s_d
   (&optional __s_d)
   (if __s_d (setq _s_d __s_d)) _s_d)
  (:s_dd
   (&optional __s_dd)
   (if __s_dd (setq _s_dd __s_dd)) _s_dd)
  (:s_ddd
   (&optional __s_ddd)
   (if __s_ddd (setq _s_ddd __s_ddd)) _s_ddd)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:ks
   (&optional __ks)
   (if __ks (setq _ks __ks)) _ks)
  (:lx
   (&optional __lx)
   (if __lx (setq _lx __lx)) _lx)
  (:ly
   (&optional __ly)
   (if __ly (setq _ly __ly)) _ly)
  (:lyaw
   (&optional __lyaw)
   (if __lyaw (setq _lyaw __lyaw)) _lyaw)
  (:serialization-length
   ()
   (+
    ;; int32 _t
    4
    ;; float32 _d
    4
    ;; float32 _d_d
    4
    ;; float32 _d_dd
    4
    ;; float32 _d_ddd
    4
    ;; float32 _s
    4
    ;; float32 _s_d
    4
    ;; float32 _s_dd
    4
    ;; float32 _s_ddd
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _yaw
    4
    ;; float32 _ks
    4
    ;; float32 _lx
    4
    ;; float32 _ly
    4
    ;; float32 _lyaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _t
       (write-long _t s)
     ;; float32 _d
       (sys::poke _d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d_d
       (sys::poke _d_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d_dd
       (sys::poke _d_dd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d_ddd
       (sys::poke _d_ddd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s
       (sys::poke _s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s_d
       (sys::poke _s_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s_dd
       (sys::poke _s_dd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s_ddd
       (sys::poke _s_ddd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ks
       (sys::poke _ks (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lx
       (sys::poke _lx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ly
       (sys::poke _ly (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lyaw
       (sys::poke _lyaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _t
     (setq _t (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _d
     (setq _d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d_d
     (setq _d_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d_dd
     (setq _d_dd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d_ddd
     (setq _d_ddd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s
     (setq _s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s_d
     (setq _s_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s_dd
     (setq _s_dd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s_ddd
     (setq _s_ddd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ks
     (setq _ks (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lx
     (setq _lx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ly
     (setq _ly (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lyaw
     (setq _lyaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get plan_msgs::PointTraj :md5sum-) "819fa2e7955aef10d3df960cc6b9813c")
(setf (get plan_msgs::PointTraj :datatype-) "plan_msgs/PointTraj")
(setf (get plan_msgs::PointTraj :definition-)
      "int32 t
float32 d
float32 d_d
float32 d_dd
float32 d_ddd
float32 s
float32 s_d
float32 s_dd
float32 s_ddd
float32 x
float32 y
float32 yaw
float32 ks
float32 lx
float32 ly
float32 lyaw

")



(provide :plan_msgs/PointTraj "819fa2e7955aef10d3df960cc6b9813c")


