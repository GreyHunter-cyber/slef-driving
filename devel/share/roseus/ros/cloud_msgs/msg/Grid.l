;; Auto-generated. Do not edit!


(when (boundp 'cloud_msgs::Grid)
  (if (not (find-package "CLOUD_MSGS"))
    (make-package "CLOUD_MSGS"))
  (shadow 'Grid (find-package "CLOUD_MSGS")))
(unless (find-package "CLOUD_MSGS::GRID")
  (make-package "CLOUD_MSGS::GRID"))

(in-package "ROS")
;;//! \htmlinclude Grid.msg.html


(defclass cloud_msgs::Grid
  :super ros::object
  :slots (_timestamp _width _height _width_step _height_step _grid_nums _grid _enabled _pos_vehicle ))

(defmethod cloud_msgs::Grid
  (:init
   (&key
    ((:timestamp __timestamp) (instance ros::time :init))
    ((:width __width) 0)
    ((:height __height) 0)
    ((:width_step __width_step) 0.0)
    ((:height_step __height_step) 0.0)
    ((:grid_nums __grid_nums) 0)
    ((:grid __grid) (make-array 0 :initial-element 0 :element-type :integer))
    ((:enabled __enabled) 0)
    ((:pos_vehicle __pos_vehicle) (instance cloud_msgs::PointXYA :init))
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _width_step (float __width_step))
   (setq _height_step (float __height_step))
   (setq _grid_nums (round __grid_nums))
   (setq _grid __grid)
   (setq _enabled (round __enabled))
   (setq _pos_vehicle __pos_vehicle)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width_step
   (&optional __width_step)
   (if __width_step (setq _width_step __width_step)) _width_step)
  (:height_step
   (&optional __height_step)
   (if __height_step (setq _height_step __height_step)) _height_step)
  (:grid_nums
   (&optional __grid_nums)
   (if __grid_nums (setq _grid_nums __grid_nums)) _grid_nums)
  (:grid
   (&optional __grid)
   (if __grid (setq _grid __grid)) _grid)
  (:enabled
   (&optional __enabled)
   (if __enabled (setq _enabled __enabled)) _enabled)
  (:pos_vehicle
   (&rest __pos_vehicle)
   (if (keywordp (car __pos_vehicle))
       (send* _pos_vehicle __pos_vehicle)
     (progn
       (if __pos_vehicle (setq _pos_vehicle (car __pos_vehicle)))
       _pos_vehicle)))
  (:serialization-length
   ()
   (+
    ;; time _timestamp
    8
    ;; int32 _width
    4
    ;; int32 _height
    4
    ;; float32 _width_step
    4
    ;; float32 _height_step
    4
    ;; int32 _grid_nums
    4
    ;; int8[] _grid
    (* 1    (length _grid)) 4
    ;; int8 _enabled
    1
    ;; cloud_msgs/PointXYA _pos_vehicle
    (send _pos_vehicle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;; int32 _width
       (write-long _width s)
     ;; int32 _height
       (write-long _height s)
     ;; float32 _width_step
       (sys::poke _width_step (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height_step
       (sys::poke _height_step (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _grid_nums
       (write-long _grid_nums s)
     ;; int8[] _grid
     (write-long (length _grid) s)
     (dotimes (i (length _grid))
       (write-byte (elt _grid i) s)
       )
     ;; int8 _enabled
       (write-byte _enabled s)
     ;; cloud_msgs/PointXYA _pos_vehicle
       (send _pos_vehicle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _width_step
     (setq _width_step (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height_step
     (setq _height_step (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _grid_nums
     (setq _grid_nums (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8[] _grid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grid (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _grid i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _grid i) 127) (setf (elt _grid i) (- (elt _grid i) 256)))
     ))
   ;; int8 _enabled
     (setq _enabled (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _enabled 127) (setq _enabled (- _enabled 256)))
   ;; cloud_msgs/PointXYA _pos_vehicle
     (send _pos_vehicle :deserialize buf ptr-) (incf ptr- (send _pos_vehicle :serialization-length))
   ;;
   self)
  )

(setf (get cloud_msgs::Grid :md5sum-) "ec789739e2a01936ea531728fbd248c4")
(setf (get cloud_msgs::Grid :datatype-) "cloud_msgs/Grid")
(setf (get cloud_msgs::Grid :definition-)
      "time       timestamp
int32      width
int32      height
float32    width_step
float32    height_step
int32      grid_nums
int8[]     grid
int8       enabled
PointXYA   pos_vehicle

================================================================================
MSG: cloud_msgs/PointXYA
float64 x
float64 y
float64 yaw  # degs

")



(provide :cloud_msgs/Grid "ec789739e2a01936ea531728fbd248c4")


