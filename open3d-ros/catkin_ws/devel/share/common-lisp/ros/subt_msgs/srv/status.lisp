; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude status-request.msg.html

(cl:defclass <status-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass status-request (<status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<status-request> is deprecated: use subt_msgs-srv:status-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <status-request>) ostream)
  "Serializes a message object of type '<status-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <status-request>) istream)
  "Deserializes a message object of type '<status-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<status-request>)))
  "Returns string type for a service object of type '<status-request>"
  "subt_msgs/statusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'status-request)))
  "Returns string type for a service object of type 'status-request"
  "subt_msgs/statusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<status-request>)))
  "Returns md5sum for a message object of type '<status-request>"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'status-request)))
  "Returns md5sum for a message object of type 'status-request"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<status-request>)))
  "Returns full string definition for message of type '<status-request>"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'status-request)))
  "Returns full string definition for message of type 'status-request"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <status-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'status-request
))
;//! \htmlinclude status-response.msg.html

(cl:defclass <status-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass status-response (<status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<status-response> is deprecated: use subt_msgs-srv:status-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:result-val is deprecated.  Use subt_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <status-response>) ostream)
  "Serializes a message object of type '<status-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <status-response>) istream)
  "Deserializes a message object of type '<status-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<status-response>)))
  "Returns string type for a service object of type '<status-response>"
  "subt_msgs/statusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'status-response)))
  "Returns string type for a service object of type 'status-response"
  "subt_msgs/statusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<status-response>)))
  "Returns md5sum for a message object of type '<status-response>"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'status-response)))
  "Returns md5sum for a message object of type 'status-response"
  "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<status-response>)))
  "Returns full string definition for message of type '<status-response>"
  (cl:format cl:nil "#Response data~%string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'status-response)))
  "Returns full string definition for message of type 'status-response"
  (cl:format cl:nil "#Response data~%string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <status-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'status-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'status)))
  'status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'status)))
  'status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'status)))
  "Returns string type for a service object of type '<status>"
  "subt_msgs/status")