; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude node_stat-request.msg.html

(cl:defclass <node_stat-request> (roslisp-msg-protocol:ros-message)
  ((count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0))
)

(cl:defclass node_stat-request (<node_stat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <node_stat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'node_stat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<node_stat-request> is deprecated: use subt_msgs-srv:node_stat-request instead.")))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <node_stat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:count-val is deprecated.  Use subt_msgs-srv:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <node_stat-request>) ostream)
  "Serializes a message object of type '<node_stat-request>"
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <node_stat-request>) istream)
  "Deserializes a message object of type '<node_stat-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<node_stat-request>)))
  "Returns string type for a service object of type '<node_stat-request>"
  "subt_msgs/node_statRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'node_stat-request)))
  "Returns string type for a service object of type 'node_stat-request"
  "subt_msgs/node_statRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<node_stat-request>)))
  "Returns md5sum for a message object of type '<node_stat-request>"
  "54387671ddc71365ab725d0391f330f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'node_stat-request)))
  "Returns md5sum for a message object of type 'node_stat-request"
  "54387671ddc71365ab725d0391f330f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<node_stat-request>)))
  "Returns full string definition for message of type '<node_stat-request>"
  (cl:format cl:nil "#Request data~%int32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'node_stat-request)))
  "Returns full string definition for message of type 'node_stat-request"
  (cl:format cl:nil "#Request data~%int32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <node_stat-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <node_stat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'node_stat-request
    (cl:cons ':count (count msg))
))
;//! \htmlinclude node_stat-response.msg.html

(cl:defclass <node_stat-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass node_stat-response (<node_stat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <node_stat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'node_stat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<node_stat-response> is deprecated: use subt_msgs-srv:node_stat-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <node_stat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:result-val is deprecated.  Use subt_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <node_stat-response>) ostream)
  "Serializes a message object of type '<node_stat-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <node_stat-response>) istream)
  "Deserializes a message object of type '<node_stat-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<node_stat-response>)))
  "Returns string type for a service object of type '<node_stat-response>"
  "subt_msgs/node_statResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'node_stat-response)))
  "Returns string type for a service object of type 'node_stat-response"
  "subt_msgs/node_statResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<node_stat-response>)))
  "Returns md5sum for a message object of type '<node_stat-response>"
  "54387671ddc71365ab725d0391f330f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'node_stat-response)))
  "Returns md5sum for a message object of type 'node_stat-response"
  "54387671ddc71365ab725d0391f330f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<node_stat-response>)))
  "Returns full string definition for message of type '<node_stat-response>"
  (cl:format cl:nil "#Response data~%string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'node_stat-response)))
  "Returns full string definition for message of type 'node_stat-response"
  (cl:format cl:nil "#Response data~%string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <node_stat-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <node_stat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'node_stat-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'node_stat)))
  'node_stat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'node_stat)))
  'node_stat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'node_stat)))
  "Returns string type for a service object of type '<node_stat>"
  "subt_msgs/node_stat")