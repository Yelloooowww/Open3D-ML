; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude ultra_info-request.msg.html

(cl:defclass <ultra_info-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ultra_info-request (<ultra_info-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ultra_info-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ultra_info-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<ultra_info-request> is deprecated: use subt_msgs-srv:ultra_info-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ultra_info-request>) ostream)
  "Serializes a message object of type '<ultra_info-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ultra_info-request>) istream)
  "Deserializes a message object of type '<ultra_info-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ultra_info-request>)))
  "Returns string type for a service object of type '<ultra_info-request>"
  "subt_msgs/ultra_infoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ultra_info-request)))
  "Returns string type for a service object of type 'ultra_info-request"
  "subt_msgs/ultra_infoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ultra_info-request>)))
  "Returns md5sum for a message object of type '<ultra_info-request>"
  "839104ea8fff48a8d67dee52c3b6127a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ultra_info-request)))
  "Returns md5sum for a message object of type 'ultra_info-request"
  "839104ea8fff48a8d67dee52c3b6127a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ultra_info-request>)))
  "Returns full string definition for message of type '<ultra_info-request>"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ultra_info-request)))
  "Returns full string definition for message of type 'ultra_info-request"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ultra_info-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ultra_info-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ultra_info-request
))
;//! \htmlinclude ultra_info-response.msg.html

(cl:defclass <ultra_info-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0))
)

(cl:defclass ultra_info-response (<ultra_info-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ultra_info-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ultra_info-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<ultra_info-response> is deprecated: use subt_msgs-srv:ultra_info-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ultra_info-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:status-val is deprecated.  Use subt_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ultra_info-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:height-val is deprecated.  Use subt_msgs-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ultra_info-response>) ostream)
  "Serializes a message object of type '<ultra_info-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ultra_info-response>) istream)
  "Deserializes a message object of type '<ultra_info-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ultra_info-response>)))
  "Returns string type for a service object of type '<ultra_info-response>"
  "subt_msgs/ultra_infoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ultra_info-response)))
  "Returns string type for a service object of type 'ultra_info-response"
  "subt_msgs/ultra_infoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ultra_info-response>)))
  "Returns md5sum for a message object of type '<ultra_info-response>"
  "839104ea8fff48a8d67dee52c3b6127a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ultra_info-response)))
  "Returns md5sum for a message object of type 'ultra_info-response"
  "839104ea8fff48a8d67dee52c3b6127a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ultra_info-response>)))
  "Returns full string definition for message of type '<ultra_info-response>"
  (cl:format cl:nil "#Response data~%bool status		# True for normal, False for danger~%float32 height~%# string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ultra_info-response)))
  "Returns full string definition for message of type 'ultra_info-response"
  (cl:format cl:nil "#Response data~%bool status		# True for normal, False for danger~%float32 height~%# string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ultra_info-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ultra_info-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ultra_info-response
    (cl:cons ':status (status msg))
    (cl:cons ':height (height msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ultra_info)))
  'ultra_info-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ultra_info)))
  'ultra_info-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ultra_info)))
  "Returns string type for a service object of type '<ultra_info>"
  "subt_msgs/ultra_info")