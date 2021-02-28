; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude int8-request.msg.html

(cl:defclass <int8-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass int8-request (<int8-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <int8-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'int8-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<int8-request> is deprecated: use subt_msgs-srv:int8-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <int8-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:data-val is deprecated.  Use subt_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <int8-request>) ostream)
  "Serializes a message object of type '<int8-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <int8-request>) istream)
  "Deserializes a message object of type '<int8-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<int8-request>)))
  "Returns string type for a service object of type '<int8-request>"
  "subt_msgs/int8Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int8-request)))
  "Returns string type for a service object of type 'int8-request"
  "subt_msgs/int8Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<int8-request>)))
  "Returns md5sum for a message object of type '<int8-request>"
  "27ffa0c9c4b8fb8492252bcad9e5c57b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'int8-request)))
  "Returns md5sum for a message object of type 'int8-request"
  "27ffa0c9c4b8fb8492252bcad9e5c57b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<int8-request>)))
  "Returns full string definition for message of type '<int8-request>"
  (cl:format cl:nil "#Request data~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'int8-request)))
  "Returns full string definition for message of type 'int8-request"
  (cl:format cl:nil "#Request data~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <int8-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <int8-request>))
  "Converts a ROS message object to a list"
  (cl:list 'int8-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude int8-response.msg.html

(cl:defclass <int8-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass int8-response (<int8-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <int8-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'int8-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<int8-response> is deprecated: use subt_msgs-srv:int8-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <int8-response>) ostream)
  "Serializes a message object of type '<int8-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <int8-response>) istream)
  "Deserializes a message object of type '<int8-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<int8-response>)))
  "Returns string type for a service object of type '<int8-response>"
  "subt_msgs/int8Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int8-response)))
  "Returns string type for a service object of type 'int8-response"
  "subt_msgs/int8Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<int8-response>)))
  "Returns md5sum for a message object of type '<int8-response>"
  "27ffa0c9c4b8fb8492252bcad9e5c57b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'int8-response)))
  "Returns md5sum for a message object of type 'int8-response"
  "27ffa0c9c4b8fb8492252bcad9e5c57b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<int8-response>)))
  "Returns full string definition for message of type '<int8-response>"
  (cl:format cl:nil "#Response data~%# string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'int8-response)))
  "Returns full string definition for message of type 'int8-response"
  (cl:format cl:nil "#Response data~%# string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <int8-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <int8-response>))
  "Converts a ROS message object to a list"
  (cl:list 'int8-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'int8)))
  'int8-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'int8)))
  'int8-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int8)))
  "Returns string type for a service object of type '<int8>"
  "subt_msgs/int8")