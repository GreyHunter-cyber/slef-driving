;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::RobotState)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'RobotState (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::ROBOTSTATE")
  (make-package "PLAN_MSGS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html


(defclass plan_msgs::RobotState
  :super ros::object
  :slots (_mition_arrived _mition_arrive_num _Speed _Azimuth _Stop _loc_fix ))

(defmethod plan_msgs::RobotState
  (:init
   (&key
    ((:mition_arrived __mition_arrived) nil)
    ((:mition_arrive_num __mition_arrive_num) 0)
    ((:Speed __Speed) 0.0)
    ((:Azimuth __Azimuth) 0.0)
    ((:Stop __Stop) nil)
    ((:loc_fix __loc_fix) 0)
    )
   (send-super :init)
   (setq _mition_arrived __mition_arrived)
   (setq _mition_arrive_num (round __mition_arrive_num))
   (setq _Speed (float __Speed))
   (setq _Azimuth (float __Azimuth))
   (setq _Stop __Stop)
   (setq _loc_fix (round __loc_fix))
   self)
  (:mition_arrived
   (&optional __mition_arrived)
   (if __mition_arrived (setq _mition_arrived __mition_arrived)) _mition_arrived)
  (:mition_arrive_num
   (&optional __mition_arrive_num)
   (if __mition_arrive_num (setq _mition_arrive_num __mition_arrive_num)) _mition_arrive_num)
  (:Speed
   (&optional __Speed)
   (if __Speed (setq _Speed __Speed)) _Speed)
  (:Azimuth
   (&optional __Azimuth)
   (if __Azimuth (setq _Azimuth __Azimuth)) _Azimuth)
  (:Stop
   (&optional __Stop)
   (if __Stop (setq _Stop __Stop)) _Stop)
  (:loc_fix
   (&optional __loc_fix)
   (if __loc_fix (setq _loc_fix __loc_fix)) _loc_fix)
  (:serialization-length
   ()
   (+
    ;; bool _mition_arrived
    1
    ;; int32 _mition_arrive_num
    4
    ;; float32 _Speed
    4
    ;; float32 _Azimuth
    4
    ;; bool _Stop
    1
    ;; int32 _loc_fix
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _mition_arrived
       (if _mition_arrived (write-byte -1 s) (write-byte 0 s))
     ;; int32 _mition_arrive_num
       (write-long _mition_arrive_num s)
     ;; float32 _Speed
       (sys::poke _Speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Azimuth
       (sys::poke _Azimuth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _Stop
       (if _Stop (write-byte -1 s) (write-byte 0 s))
     ;; int32 _loc_fix
       (write-long _loc_fix s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _mition_arrived
     (setq _mition_arrived (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _mition_arrive_num
     (setq _mition_arrive_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _Speed
     (setq _Speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Azimuth
     (setq _Azimuth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _Stop
     (setq _Stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _loc_fix
     (setq _loc_fix (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get plan_msgs::RobotState :md5sum-) "ab96e98e2f0ba56ad199d5f5d6baa6be")
(setf (get plan_msgs::RobotState :datatype-) "plan_msgs/RobotState")
(setf (get plan_msgs::RobotState :definition-)
      "#topic: /RobotState
bool mition_arrived # arrived
int32 mition_arrive_num # arrived whitch misstion point
float32 Speed
float32 Azimuth
bool  Stop     # pause or stop flag enable
int32 loc_fix  # Positioning quality 0: miss; 1: fixed; 2: float;

")



(provide :plan_msgs/RobotState "ab96e98e2f0ba56ad199d5f5d6baa6be")


