
(cl:in-package :asdf)

(defsystem "lslidar_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LslidarC16Sweep" :depends-on ("_package_LslidarC16Sweep"))
    (:file "_package_LslidarC16Sweep" :depends-on ("_package"))
    (:file "LslidarC32Sweep" :depends-on ("_package_LslidarC32Sweep"))
    (:file "_package_LslidarC32Sweep" :depends-on ("_package"))
    (:file "LslidarPacket" :depends-on ("_package_LslidarPacket"))
    (:file "_package_LslidarPacket" :depends-on ("_package"))
    (:file "LslidarPoint" :depends-on ("_package_LslidarPoint"))
    (:file "_package_LslidarPoint" :depends-on ("_package"))
    (:file "LslidarScan" :depends-on ("_package_LslidarScan"))
    (:file "_package_LslidarScan" :depends-on ("_package"))
    (:file "LslidarScanUnified" :depends-on ("_package_LslidarScanUnified"))
    (:file "_package_LslidarScanUnified" :depends-on ("_package"))
  ))