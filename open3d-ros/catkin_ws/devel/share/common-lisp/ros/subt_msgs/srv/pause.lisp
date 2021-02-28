; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude pause-request.msg.html

(cl:defclass <pause-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass pause-request (<pause-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pause-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pause-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<pause-request> is deprecated: use subt_msgs-srv:pause-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pause-request>) ostream)
  "Serializes a message object of type '<pause-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pause-request>) istream)
  "Deserializes a message object of type '<pause-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pause-request>)))
  "Returns string type for a service object of type '<pause-request>"
  "subt_msgs/pauseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pause-request)))
  "Returns string type for a service object of type 'pause-request"
  "subt_msgs/pauseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pause-request>)))
  "Returns md5sum for a message object of type '<pause-request>"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pause-request)))
  "Returns md5sum for a message object of type 'pause-request"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pause-request>)))
  "Returns full string definition for message of type '<pause-request>"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pause-request)))
  "Returns full string definition for message of type 'pause-request"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pause-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pause-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pause-request
))
;//! \htmlinclude pause-response.msg.html

(cl:defclass <pause-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass pause-response (<pause-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pause-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pause-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<pause-response> is deprecated: use subt_msgs-srv:pause-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <pause-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:result-val is deprecated.  Use subt_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pause-response>) ostream)
  "Serializes a message object of type '<pause-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pause-response>) istream)
  "Deserializes a message object of type '<pause-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pause-response>)))
  "Returns string type for a service object of type '<pause-response>"
  "subt_msgs/pauseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pause-response)))
  "Returns string type for a service object of type 'pause-response"
  "subt_msgs/pauseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pause-response>)))
  "Returns md5sum for a message object of type '<pause-response>"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pause-response)))
  "Returns md5sum for a message object of type 'pause-response"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pause-response>)))
  "Returns full string definition for message of type '<pause-response>"
  (cl:format cl:nil "#Response data~%string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pause-response)))
  "Returns full string definition for message of type 'pause-response"
  (cl:format cl:nil "#Response data~%string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pause-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pause-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pause-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pause)))
  'pause-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pause)))
  'pause-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pause)))
  "Returns string type for a service object of type '<pause>"
  "subt_msgs/pause")