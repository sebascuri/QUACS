; Auto-generated. Do not edit!


(cl:in-package ardrone_control-srv)


;//! \htmlinclude Signal-request.msg.html

(cl:defclass <Signal-request> (roslisp-msg-protocol:ros-message)
  ((signal
    :reader signal
    :initarg :signal
    :type cl:string
    :initform "")
   (direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform "")
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (dt
    :reader dt
    :initarg :dt
    :type cl:float
    :initform 0.0)
   (f
    :reader f
    :initarg :f
    :type cl:float
    :initform 0.0))
)

(cl:defclass Signal-request (<Signal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Signal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Signal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_control-srv:<Signal-request> is deprecated: use ardrone_control-srv:Signal-request instead.")))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <Signal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_control-srv:signal-val is deprecated.  Use ardrone_control-srv:signal instead.")
  (signal m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Signal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_control-srv:direction-val is deprecated.  Use ardrone_control-srv:direction instead.")
  (direction m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Signal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_control-srv:time-val is deprecated.  Use ardrone_control-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <Signal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_control-srv:dt-val is deprecated.  Use ardrone_control-srv:dt instead.")
  (dt m))

(cl:ensure-generic-function 'f-val :lambda-list '(m))
(cl:defmethod f-val ((m <Signal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_control-srv:f-val is deprecated.  Use ardrone_control-srv:f instead.")
  (f m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Signal-request>) ostream)
  "Serializes a message object of type '<Signal-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'signal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'signal))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'f))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Signal-request>) istream)
  "Deserializes a message object of type '<Signal-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'signal) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'signal) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Signal-request>)))
  "Returns string type for a service object of type '<Signal-request>"
  "ardrone_control/SignalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Signal-request)))
  "Returns string type for a service object of type 'Signal-request"
  "ardrone_control/SignalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Signal-request>)))
  "Returns md5sum for a message object of type '<Signal-request>"
  "c331ab0422e603181748d1d753651531")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Signal-request)))
  "Returns md5sum for a message object of type 'Signal-request"
  "c331ab0422e603181748d1d753651531")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Signal-request>)))
  "Returns full string definition for message of type '<Signal-request>"
  (cl:format cl:nil "string signal~%string direction~%float64 time~%float64 dt~%float64 f~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Signal-request)))
  "Returns full string definition for message of type 'Signal-request"
  (cl:format cl:nil "string signal~%string direction~%float64 time~%float64 dt~%float64 f~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Signal-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'signal))
     4 (cl:length (cl:slot-value msg 'direction))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Signal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Signal-request
    (cl:cons ':signal (signal msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':time (time msg))
    (cl:cons ':dt (dt msg))
    (cl:cons ':f (f msg))
))
;//! \htmlinclude Signal-response.msg.html

(cl:defclass <Signal-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Signal-response (<Signal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Signal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Signal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_control-srv:<Signal-response> is deprecated: use ardrone_control-srv:Signal-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Signal-response>) ostream)
  "Serializes a message object of type '<Signal-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Signal-response>) istream)
  "Deserializes a message object of type '<Signal-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Signal-response>)))
  "Returns string type for a service object of type '<Signal-response>"
  "ardrone_control/SignalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Signal-response)))
  "Returns string type for a service object of type 'Signal-response"
  "ardrone_control/SignalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Signal-response>)))
  "Returns md5sum for a message object of type '<Signal-response>"
  "c331ab0422e603181748d1d753651531")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Signal-response)))
  "Returns md5sum for a message object of type 'Signal-response"
  "c331ab0422e603181748d1d753651531")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Signal-response>)))
  "Returns full string definition for message of type '<Signal-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Signal-response)))
  "Returns full string definition for message of type 'Signal-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Signal-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Signal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Signal-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Signal)))
  'Signal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Signal)))
  'Signal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Signal)))
  "Returns string type for a service object of type '<Signal>"
  "ardrone_control/Signal")