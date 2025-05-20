; Auto-generated. Do not edit!


(cl:in-package lslidar_msgs-srv)


;//! \htmlinclude lslidar_control-request.msg.html

(cl:defclass <lslidar_control-request> (roslisp-msg-protocol:ros-message)
  ((LaserControl
    :reader LaserControl
    :initarg :LaserControl
    :type cl:integer
    :initform 0))
)

(cl:defclass lslidar_control-request (<lslidar_control-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lslidar_control-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lslidar_control-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lslidar_msgs-srv:<lslidar_control-request> is deprecated: use lslidar_msgs-srv:lslidar_control-request instead.")))

(cl:ensure-generic-function 'LaserControl-val :lambda-list '(m))
(cl:defmethod LaserControl-val ((m <lslidar_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lslidar_msgs-srv:LaserControl-val is deprecated.  Use lslidar_msgs-srv:LaserControl instead.")
  (LaserControl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lslidar_control-request>) ostream)
  "Serializes a message object of type '<lslidar_control-request>"
  (cl:let* ((signed (cl:slot-value msg 'LaserControl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lslidar_control-request>) istream)
  "Deserializes a message object of type '<lslidar_control-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LaserControl) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lslidar_control-request>)))
  "Returns string type for a service object of type '<lslidar_control-request>"
  "lslidar_msgs/lslidar_controlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lslidar_control-request)))
  "Returns string type for a service object of type 'lslidar_control-request"
  "lslidar_msgs/lslidar_controlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lslidar_control-request>)))
  "Returns md5sum for a message object of type '<lslidar_control-request>"
  "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lslidar_control-request)))
  "Returns md5sum for a message object of type 'lslidar_control-request"
  "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lslidar_control-request>)))
  "Returns full string definition for message of type '<lslidar_control-request>"
  (cl:format cl:nil "int32 LaserControl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lslidar_control-request)))
  "Returns full string definition for message of type 'lslidar_control-request"
  (cl:format cl:nil "int32 LaserControl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lslidar_control-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lslidar_control-request>))
  "Converts a ROS message object to a list"
  (cl:list 'lslidar_control-request
    (cl:cons ':LaserControl (LaserControl msg))
))
;//! \htmlinclude lslidar_control-response.msg.html

(cl:defclass <lslidar_control-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass lslidar_control-response (<lslidar_control-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lslidar_control-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lslidar_control-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lslidar_msgs-srv:<lslidar_control-response> is deprecated: use lslidar_msgs-srv:lslidar_control-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <lslidar_control-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lslidar_msgs-srv:status-val is deprecated.  Use lslidar_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lslidar_control-response>) ostream)
  "Serializes a message object of type '<lslidar_control-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lslidar_control-response>) istream)
  "Deserializes a message object of type '<lslidar_control-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lslidar_control-response>)))
  "Returns string type for a service object of type '<lslidar_control-response>"
  "lslidar_msgs/lslidar_controlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lslidar_control-response)))
  "Returns string type for a service object of type 'lslidar_control-response"
  "lslidar_msgs/lslidar_controlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lslidar_control-response>)))
  "Returns md5sum for a message object of type '<lslidar_control-response>"
  "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lslidar_control-response)))
  "Returns md5sum for a message object of type 'lslidar_control-response"
  "0d2a2bc3bb3a258db25b2d8c05ffa5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lslidar_control-response>)))
  "Returns full string definition for message of type '<lslidar_control-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lslidar_control-response)))
  "Returns full string definition for message of type 'lslidar_control-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lslidar_control-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lslidar_control-response>))
  "Converts a ROS message object to a list"
  (cl:list 'lslidar_control-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'lslidar_control)))
  'lslidar_control-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'lslidar_control)))
  'lslidar_control-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lslidar_control)))
  "Returns string type for a service object of type '<lslidar_control>"
  "lslidar_msgs/lslidar_control")