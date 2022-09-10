;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::Traj)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'Traj (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::TRAJ")
  (make-package "PLAN_MSGS::TRAJ"))

(in-package "ROS")
;;//! \htmlinclude Traj.msg.html


(defclass plan_msgs::Traj
  :super ros::object
  :slots (_num _points ))

(defmethod plan_msgs::Traj
  (:init
   (&key
    ((:num __num) 0)
    ((:points __points) (let (r) (dotimes (i 0) (push (instance plan_msgs::PointTraj :init) r)) r))
    )
   (send-super :init)
   (setq _num (round __num))
   (setq _points __points)
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; int32 _num
    4
    ;; plan_msgs/PointTraj[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num
       (write-long _num s)
     ;; plan_msgs/PointTraj[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; plan_msgs/PointTraj[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance plan_msgs::PointTraj :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get plan_msgs::Traj :md5sum-) "490cb3b1cf624497cbfba91636b0dd01")
(setf (get plan_msgs::Traj :datatype-) "plan_msgs/Traj")
(setf (get plan_msgs::Traj :definition-)
      "int32     num
PointTraj[]  points

================================================================================
MSG: plan_msgs/PointTraj
int32 t
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



(provide :plan_msgs/Traj "490cb3b1cf624497cbfba91636b0dd01")


