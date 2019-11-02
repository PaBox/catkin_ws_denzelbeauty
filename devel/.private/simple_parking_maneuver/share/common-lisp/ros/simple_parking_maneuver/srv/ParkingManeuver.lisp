; Auto-generated. Do not edit!


(cl:in-package simple_parking_maneuver-srv)


;//! \htmlinclude ParkingManeuver-request.msg.html

(cl:defclass <ParkingManeuver-request> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform ""))
)

(cl:defclass ParkingManeuver-request (<ParkingManeuver-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParkingManeuver-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParkingManeuver-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_parking_maneuver-srv:<ParkingManeuver-request> is deprecated: use simple_parking_maneuver-srv:ParkingManeuver-request instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <ParkingManeuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_parking_maneuver-srv:direction-val is deprecated.  Use simple_parking_maneuver-srv:direction instead.")
  (direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParkingManeuver-request>) ostream)
  "Serializes a message object of type '<ParkingManeuver-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParkingManeuver-request>) istream)
  "Deserializes a message object of type '<ParkingManeuver-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParkingManeuver-request>)))
  "Returns string type for a service object of type '<ParkingManeuver-request>"
  "simple_parking_maneuver/ParkingManeuverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParkingManeuver-request)))
  "Returns string type for a service object of type 'ParkingManeuver-request"
  "simple_parking_maneuver/ParkingManeuverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParkingManeuver-request>)))
  "Returns md5sum for a message object of type '<ParkingManeuver-request>"
  "7a31466994d8db49e4cf1556345a8002")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParkingManeuver-request)))
  "Returns md5sum for a message object of type 'ParkingManeuver-request"
  "7a31466994d8db49e4cf1556345a8002")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParkingManeuver-request>)))
  "Returns full string definition for message of type '<ParkingManeuver-request>"
  (cl:format cl:nil "string direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParkingManeuver-request)))
  "Returns full string definition for message of type 'ParkingManeuver-request"
  (cl:format cl:nil "string direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParkingManeuver-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'direction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParkingManeuver-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ParkingManeuver-request
    (cl:cons ':direction (direction msg))
))
;//! \htmlinclude ParkingManeuver-response.msg.html

(cl:defclass <ParkingManeuver-response> (roslisp-msg-protocol:ros-message)
  ((info
    :reader info
    :initarg :info
    :type cl:string
    :initform ""))
)

(cl:defclass ParkingManeuver-response (<ParkingManeuver-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParkingManeuver-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParkingManeuver-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_parking_maneuver-srv:<ParkingManeuver-response> is deprecated: use simple_parking_maneuver-srv:ParkingManeuver-response instead.")))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <ParkingManeuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_parking_maneuver-srv:info-val is deprecated.  Use simple_parking_maneuver-srv:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParkingManeuver-response>) ostream)
  "Serializes a message object of type '<ParkingManeuver-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParkingManeuver-response>) istream)
  "Deserializes a message object of type '<ParkingManeuver-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParkingManeuver-response>)))
  "Returns string type for a service object of type '<ParkingManeuver-response>"
  "simple_parking_maneuver/ParkingManeuverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParkingManeuver-response)))
  "Returns string type for a service object of type 'ParkingManeuver-response"
  "simple_parking_maneuver/ParkingManeuverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParkingManeuver-response>)))
  "Returns md5sum for a message object of type '<ParkingManeuver-response>"
  "7a31466994d8db49e4cf1556345a8002")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParkingManeuver-response)))
  "Returns md5sum for a message object of type 'ParkingManeuver-response"
  "7a31466994d8db49e4cf1556345a8002")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParkingManeuver-response>)))
  "Returns full string definition for message of type '<ParkingManeuver-response>"
  (cl:format cl:nil "string info~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParkingManeuver-response)))
  "Returns full string definition for message of type 'ParkingManeuver-response"
  (cl:format cl:nil "string info~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParkingManeuver-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParkingManeuver-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ParkingManeuver-response
    (cl:cons ':info (info msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ParkingManeuver)))
  'ParkingManeuver-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ParkingManeuver)))
  'ParkingManeuver-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParkingManeuver)))
  "Returns string type for a service object of type '<ParkingManeuver>"
  "simple_parking_maneuver/ParkingManeuver")