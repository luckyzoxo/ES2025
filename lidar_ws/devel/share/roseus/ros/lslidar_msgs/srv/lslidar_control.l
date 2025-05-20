;; Auto-generated. Do not edit!


(when (boundp 'lslidar_msgs::lslidar_control)
  (if (not (find-package "LSLIDAR_MSGS"))
    (make-package "LSLIDAR_MSGS"))
  (shadow 'lslidar_control (find-package "LSLIDAR_MSGS")))
(unless (find-package "LSLIDAR_MSGS::LSLIDAR_CONTROL")
  (make-package "LSLIDAR_MSGS::LSLIDAR_CONTROL"))
(unless (find-package "LSLIDAR_MSGS::LSLIDAR_CONTROLREQUEST")
  (make-package "LSLIDAR_MSGS::LSLIDAR_CONTROLREQUEST"))
(unless (find-package "LSLIDAR_MSGS::LSLIDAR_CONTROLRESPONSE")
  (make-package "LSLIDAR_MSGS::LSLIDAR_CONTROLRESPONSE"))

(in-package "ROS")





(defclass lslidar_msgs::lslidar_controlRequest
  :super ros::object
  :slots (_LaserControl ))

(defmethod lslidar_msgs::lslidar_controlRequest
  (:init
   (&key
    ((:LaserControl __LaserControl) 0)
    )
   (send-super :init)
   (setq _LaserControl (round __LaserControl))
   self)
  (:LaserControl
   (&optional __LaserControl)
   (if __LaserControl (setq _LaserControl __LaserControl)) _LaserControl)
  (:serialization-length
   ()
   (+
    ;; int32 _LaserControl
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _LaserControl
       (write-long _LaserControl s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _LaserControl
     (setq _LaserControl (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass lslidar_msgs::lslidar_controlResponse
  :super ros::object
  :slots (_status ))

(defmethod lslidar_msgs::lslidar_controlResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass lslidar_msgs::lslidar_control
  :super ros::object
  :slots ())

(setf (get lslidar_msgs::lslidar_control :md5sum-) "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(setf (get lslidar_msgs::lslidar_control :datatype-) "lslidar_msgs/lslidar_control")
(setf (get lslidar_msgs::lslidar_control :request) lslidar_msgs::lslidar_controlRequest)
(setf (get lslidar_msgs::lslidar_control :response) lslidar_msgs::lslidar_controlResponse)

(defmethod lslidar_msgs::lslidar_controlRequest
  (:response () (instance lslidar_msgs::lslidar_controlResponse :init)))

(setf (get lslidar_msgs::lslidar_controlRequest :md5sum-) "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(setf (get lslidar_msgs::lslidar_controlRequest :datatype-) "lslidar_msgs/lslidar_controlRequest")
(setf (get lslidar_msgs::lslidar_controlRequest :definition-)
      "int32 LaserControl
---
bool status

")

(setf (get lslidar_msgs::lslidar_controlResponse :md5sum-) "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(setf (get lslidar_msgs::lslidar_controlResponse :datatype-) "lslidar_msgs/lslidar_controlResponse")
(setf (get lslidar_msgs::lslidar_controlResponse :definition-)
      "int32 LaserControl
---
bool status

")



(provide :lslidar_msgs/lslidar_control "0d2a2bc3bb3a258db25b2d8c05ffa5d1")


