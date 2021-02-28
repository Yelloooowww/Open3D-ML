; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude start-request.msg.html

(cl:defclass <start-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass start-request (<start-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <start-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'start-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<start-request> is deprecated: use subt_msgs-srv:start-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <start-request>) ostream)
  "Serializes a message object of type '<start-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <start-request>) istream)
  "Deserializes a message object of type '<start-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<start-request>)))
  "Returns string type for a service object of type '<start-request>"
  "subt_msgs/startRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'start-request)))
  "Returns string type for a service object of type 'start-request"
  "subt_msgs/startRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<start-request>)))
  "Returns md5sum for a message object of type '<start-request>"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'start-request)))
  "Returns md5sum for a message object of type 'start-request"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<start-request>)))
  "Returns full string definition for message of type '<start-request>"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'start-request)))
  "Returns full string definition for message of type 'start-request"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <start-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <start-request>))
  "Converts a ROS message object to a list"
  (cl:list 'start-request
))
;//! \htmlinclude start-response.msg.html

(cl:defclass <start-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass start-response (<start-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <start-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'start-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<start-response> is deprecated: use subt_msgs-srv:start-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <start-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:result-val is deprecated.  Use subt_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <start-response>) ostream)
  "Serializes a message object of type '<start-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <start-response>) istream)
  "Deserializes a message object of type '<start-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<start-response>)))
  "Returns string type for a service object of type '<start-response>"
  "subt_msgs/startResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'start-response)))
  "Returns string type for a service object of type 'start-response"
  "subt_msgs/startResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<start-response>)))
  "Returns md5sum for a message object of type '<start-response>"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'start-response)))
  "Returns md5sum for a message object of type 'start-response"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<start-response>)))
  "Returns full string definition for message of type '<start-response>"
  (cl:format cl:nil "#Response data~%string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'start-response)))
  "Returns full string definition for message of type 'start-response"
  (cl:format cl:nil "#Response data~%string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <start-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <start-response>))
  "Converts a ROS message object to a list"
  (cl:list 'start-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'start)))
  'start-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'start)))
  'start-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'start)))
  "Returns string type for a service object of type '<start>"
  "subt_msgs/start")