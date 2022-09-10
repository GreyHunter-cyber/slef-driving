;; Auto-generated. Do not edit!


(when (boundp 'plan_msgs::Grid)
  (if (not (find-package "PLAN_MSGS"))
    (make-package "PLAN_MSGS"))
  (shadow 'Grid (find-package "PLAN_MSGS")))
(unless (find-package "PLAN_MSGS::GRID")
  (make-package "PLAN_MSGS::GRID"))

(in-package "ROS")
;;//! \htmlinclude Grid.msg.html


(defclass plan_msgs::Grid
  :super ros::object
  :slots (_stamp _width _height _size _value _d_width _d_height ))

(defmethod plan_msgs::Grid
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:width __width) 0)
    ((:height __height) 0)
    ((:size __size) 0)
    ((:value __value) (make-array 0 :initial-element 0 :element-type :integer))
    ((:d_width __d_width) 0.0)
    ((:d_height __d_height) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _size (round __size))
   (setq _value __value)
   (setq _d_width (float __d_width))
   (setq _d_height (float __d_height))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:d_width
   (&optional __d_width)
   (if __d_width (setq _d_width __d_width)) _d_width)
  (:d_height
   (&optional __d_height)
   (if __d_height (setq _d_height __d_height)) _d_height)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; int32 _width
    4
    ;; int32 _height
    4
    ;; int32 _size
    4
    ;; int32[] _value
    (* 4    (length _value)) 4
    ;; float32 _d_width
    4
    ;; float32 _d_height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _width
       (write-long _width s)
     ;; int32 _height
       (write-long _height s)
     ;; int32 _size
       (write-long _size s)
     ;; int32[] _value
     (write-long (length _value) s)
     (dotimes (i (length _value))
       (write-long (elt _value i) s)
       )
     ;; float32 _d_width
       (sys::poke _d_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d_height
       (sys::poke _d_height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _value
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _value (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _value i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32 _d_width
     (setq _d_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d_height
     (setq _d_height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get plan_msgs::Grid :md5sum-) "84268bd882d6a8d85ea94e3017e923c7")
(setf (get plan_msgs::Grid :datatype-) "plan_msgs/Grid")
(setf (get plan_msgs::Grid :definition-)
      "time stamp                                                      #当前时间
int32 width                                                     #宽度分片数，左→右
int32 height                                                    #远近分片数，近→远
int32 size
int32[] value                                                  #路面分片的矩阵
float32 d_width                                                 #格网分辨率
float32 d_height                                                #格网分辨率

")



(provide :plan_msgs/Grid "84268bd882d6a8d85ea94e3017e923c7")


