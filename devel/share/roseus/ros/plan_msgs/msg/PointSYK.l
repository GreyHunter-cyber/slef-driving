;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::PointSYK)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'PointSYK (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::POINTSYK")
  (make-package "PLAN_MSGS::POINTSYK"))

(in-package "ROS")
;;//! \htmlinclude PointSYK.msg.html


(defclass plan_msgs::PointSYK
  :super ros::object
  :slots (_x _y _s _yaw _ks ))

(defmethod plan_msgs::PointSYK
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:s __s) 0.0)
    ((:yaw __yaw) 0.0)
    ((:ks __ks) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _s (float __s))
   (setq _yaw (float __yaw))
   (setq _ks (float __ks))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:ks
   (&optional __ks)
   (if __ks (setq _ks __ks)) _ks)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _s
    4
    ;; float32 _yaw
    4
    ;; float32 _ks
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s
       (sys::poke _s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ks
       (sys::poke _ks (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s
     (setq _s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ks
     (setq _ks (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get plan_msgs::PointSYK :md5sum-) "38aea89911d068b911d894eef5ebbb8c")
(setf (get plan_msgs::PointSYK :datatype-) "plan_msgs/PointSYK")
(setf (get plan_msgs::PointSYK :definition-)
      "float32 x
float32 y
float32 s
float32 yaw  # rad
float32 ks

")



(provide :plan_msgs/PointSYK "38aea89911d068b911d894eef5ebbb8c")


