;; Auto-generated. Do not edit!


(when (boundp 'lslidar_msgs::LslidarScanUnified)
  (if (not (find-package "LSLIDAR_MSGS"))
    (make-package "LSLIDAR_MSGS"))
  (shadow 'LslidarScanUnified (find-package "LSLIDAR_MSGS")))
(unless (find-package "LSLIDAR_MSGS::LSLIDARSCANUNIFIED")
  (make-package "LSLIDAR_MSGS::LSLIDARSCANUNIFIED"))

(in-package "ROS")
;;//! \htmlinclude LslidarScanUnified.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lslidar_msgs::LslidarScanUnified
  :super ros::object
  :slots (_header _packets ))

(defmethod lslidar_msgs::LslidarScanUnified
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:packets __packets) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _packets __packets)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:packets
   (&rest __packets)
   (if (keywordp (car __packets))
       (send* _packets __packets)
     (progn
       (if __packets (setq _packets (car __packets)))
       _packets)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; lslidar_msgs/LslidarPacket[] _packets
    (apply #'+ (send-all _packets :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; lslidar_msgs/LslidarPacket[] _packets
     (write-long (length _packets) s)
     (dolist (elem _packets)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; lslidar_msgs/LslidarPacket[] _packets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _packets (let (r) (dotimes (i n) (push (instance lslidar_msgs::LslidarPacket :init) r)) r))
     (dolist (elem- _packets)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get lslidar_msgs::LslidarScanUnified :md5sum-) "50804fc9533a0e579e6322c04ae70566")
(setf (get lslidar_msgs::LslidarScanUnified :datatype-) "lslidar_msgs/LslidarScanUnified")
(setf (get lslidar_msgs::LslidarScanUnified :definition-)
      "#lslidar  scan packets

Header           header         #standard ros message header

LslidarPacket[] packets        #vector of raw packets

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: lslidar_msgs/LslidarPacket
# Raw Leishen LIDAR packet.

time stamp              # packet timestamp
uint8[1206] data        # packet contents


")



(provide :lslidar_msgs/LslidarScanUnified "50804fc9533a0e579e6322c04ae70566")


