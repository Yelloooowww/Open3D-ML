; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude int32-request.msg.html

(cl:defclass <int32-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass int32-request (<int32-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <int32-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'int32-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<int32-request> is deprecated: use subt_msgs-srv:int32-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <int32-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:data-val is deprecated.  Use subt_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <int32-request>) ostream)
  "Serializes a message object of type '<int32-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <int32-request>) istream)
  "Deserializes a message object of type '<int32-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<int32-request>)))
  "Returns string type for a service object of type '<int32-request>"
  "subt_msgs/int32Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int32-request)))
  "Returns string type for a service object of type 'int32-request"
  "subt_msgs/int32Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<int32-request>)))
  "Returns md5sum for a message object of type '<int32-request>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'int32-request)))
  "Returns md5sum for a message object of type 'int32-request"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<int32-request>)))
  "Returns full string definition for message of type '<int32-request>"
  (cl:format cl:nil "#Request data~%int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'int32-request)))
  "Returns full string definition for message of type 'int32-request"
  (cl:format cl:nil "#Request data~%int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <int32-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <int32-request>))
  "Converts a ROS message object to a list"
  (cl:list 'int32-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude int32-response.msg.html

(cl:defclass <int32-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass int32-response (<int32-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <int32-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'int32-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<int32-response> is deprecated: use subt_msgs-srv:int32-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <int32-response>) ostream)
  "Serializes a message object of type '<int32-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <int32-response>) istream)
  "Deserializes a message object of type '<int32-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<int32-response>)))
  "Returns string type for a service object of type '<int32-response>"
  "subt_msgs/int32Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int32-response)))
  "Returns string type for a service object of type 'int32-response"
  "subt_msgs/int32Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<int32-response>)))
  "Returns md5sum for a message object of type '<int32-response>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'int32-response)))
  "Returns md5sum for a message object of type 'int32-response"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<int32-response>)))
  "Returns full string definition for message of type '<int32-response>"
  (cl:format cl:nil "#Response data~%# string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'int32-response)))
  "Returns full string definition for message of type 'int32-response"
  (cl:format cl:nil "#Response data~%# string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <int32-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <int32-response>))
  "Converts a ROS message object to a list"
  (cl:list 'int32-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'int32)))
  'int32-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'int32)))
  'int32-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int32)))
  "Returns string type for a service object of type '<int32>"
  "subt_msgs/int32")