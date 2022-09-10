;; Auto-generated. Do not edit!


(when (boundp 'carTop::carTop)
  (if (not (find-package "CARTOP"))
    (make-package "CARTOP"))
  (shadow 'carTop (find-package "CARTOP")))
(unless (find-package "CARTOP::CARTOP")
  (make-package "CARTOP::CARTOP"))

(in-package "ROS")
;;//! \htmlinclude carTop.msg.html


(defclass carTop::carTop
  :super ros::object
  :slots (_back_wheel_speed _turn_angle _battery_level _error_flag ))

(defmethod carTop::carTop
  (:init
   (&key
    ((:back_wheel_speed __back_wheel_speed) 0.0)
    ((:turn_angle __turn_angle) 0.0)
    ((:battery_level __battery_level) 0)
    ((:error_flag __error_flag) 0)
    )
   (send-super :init)
   (setq _back_wheel_speed (float __back_wheel_speed))
   (setq _turn_angle (float __turn_angle))
   (setq _battery_level (round __battery_level))
   (setq _error_flag (round __error_flag))
   self)
  (:back_wheel_speed
   (&optional __back_wheel_speed)
   (if __back_wheel_speed (setq _back_wheel_speed __back_wheel_speed)) _back_wheel_speed)
  (:turn_angle
   (&optional __turn_angle)
   (if __turn_angle (setq _turn_angle __turn_angle)) _turn_angle)
  (:battery_level
   (&optional __battery_level)
   (if __battery_level (setq _battery_level __battery_level)) _battery_level)
  (:error_flag
   (&optional __error_flag)
   (if __error_flag (setq _error_flag __error_flag)) _error_flag)
  (:serialization-length
   ()
   (+
    ;; float64 _back_wheel_speed
    8
    ;; float64 _turn_angle
    8
    ;; char _battery_level
    1
    ;; char _error_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _back_wheel_speed
       (sys::poke _back_wheel_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _turn_angle
       (sys::poke _turn_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; char _battery_level
       (write-byte _battery_level s)
     ;; char _error_flag
       (write-byte _error_flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _back_wheel_speed
     (setq _back_wheel_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _turn_angle
     (setq _turn_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; char _battery_level
     (setq _battery_level (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; char _error_flag
     (setq _error_flag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get carTop::carTop :md5sum-) "2399edfa2a8160084c724032695e45ee")
(setf (get carTop::carTop :datatype-) "carTop/carTop")
(setf (get carTop::carTop :definition-)
      "float64 back_wheel_speed
float64 turn_angle
char battery_level
char error_flag

")



(provide :carTop/carTop "2399edfa2a8160084c724032695e45ee")


