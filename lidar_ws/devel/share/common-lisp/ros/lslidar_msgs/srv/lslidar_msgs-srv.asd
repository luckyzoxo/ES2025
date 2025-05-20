
(cl:in-package :asdf)

(defsystem "lslidar_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "lslidar_control" :depends-on ("_package_lslidar_control"))
    (:file "_package_lslidar_control" :depends-on ("_package"))
  ))