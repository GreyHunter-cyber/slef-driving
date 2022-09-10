;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::Path)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'Path (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::PATH")
  (make-package "PLAN_MSGS::PATH"))

(in-package "ROS")
;;//! \htmlinclude Path.msg.html


(defclass plan_msgs::Path
  :super ros::object
  :slots (_num _points ))

(defmethod plan_msgs::Path
  (:init
   (&key
    ((:num __num) 0)
    ((:points __points) (let (r) (dotimes (i 0) (push (instance plan_msgs::PointSYK :init) r)) r))
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
    ;; plan_msgs/PointSYK[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num
       (write-long _num s)
     ;; plan_msgs/PointSYK[] _points
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
   ;; plan_msgs/PointSYK[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance plan_msgs::PointSYK :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get plan_msgs::Path :md5sum-) "f43f5f3b99308fc26e35370fe5eb3f6f")
(setf (get plan_msgs::Path :datatype-) "plan_msgs/Path")
(setf (get plan_msgs::Path :definition-)
      "int32     num
PointSYK[]  points

================================================================================
MSG: plan_msgs/PointSYK
float32 x
float32 y
float32 s
float32 yaw  # rad
float32 ks

")



(provide :plan_msgs/Path "f43f5f3b99308fc26e35370fe5eb3f6f")


