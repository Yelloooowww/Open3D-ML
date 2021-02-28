; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude frame_switch-request.msg.html

(cl:defclass <frame_switch-request> (roslisp-msg-protocol:ros-message)
  ((frame
    :reader frame
    :initarg :frame
    :type cl:string
    :initform ""))
)

(cl:defclass frame_switch-request (<frame_switch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <frame_switch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'frame_switch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<frame_switch-request> is deprecated: use subt_msgs-srv:frame_switch-request instead.")))

(cl:ensure-generic-function 'frame-val :lambda-list '(m))
(cl:defmethod frame-val ((m <frame_switch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:frame-val is deprecated.  Use subt_msgs-srv:frame instead.")
  (frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <frame_switch-request>) ostream)
  "Serializes a message object of type '<frame_switch-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <frame_switch-request>) istream)
  "Deserializes a message object of type '<frame_switch-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<frame_switch-request>)))
  "Returns string type for a service object of type '<frame_switch-request>"
  "subt_msgs/frame_switchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'frame_switch-request)))
  "Returns string type for a service object of type 'frame_switch-request"
  "subt_msgs/frame_switchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<frame_switch-request>)))
  "Returns md5sum for a message object of type '<frame_switch-request>"
  "50cdb9fa30e06e199809b1d13df3e46b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'frame_switch-request)))
  "Returns md5sum for a message object of type 'frame_switch-request"
  "50cdb9fa30e06e199809b1d13df3e46b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<frame_switch-request>)))
  "Returns full string definition for message of type '<frame_switch-request>"
  (cl:format cl:nil "#Request data~%string frame # for switch frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'frame_switch-request)))
  "Returns full string definition for message of type 'frame_switch-request"
  (cl:format cl:nil "#Request data~%string frame # for switch frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <frame_switch-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <frame_switch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'frame_switch-request
    (cl:cons ':frame (frame msg))
))
;//! \htmlinclude frame_switch-response.msg.html

(cl:defclass <frame_switch-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass frame_switch-response (<frame_switch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <frame_switch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'frame_switch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<frame_switch-response> is deprecated: use subt_msgs-srv:frame_switch-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <frame_switch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:result-val is deprecated.  Use subt_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <frame_switch-response>) ostream)
  "Serializes a message object of type '<frame_switch-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <frame_switch-response>) istream)
  "Deserializes a message object of type '<frame_switch-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<frame_switch-response>)))
  "Returns string type for a service object of type '<frame_switch-response>"
  "subt_msgs/frame_switchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'frame_switch-response)))
  "Returns string type for a service object of type 'frame_switch-response"
  "subt_msgs/frame_switchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<frame_switch-response>)))
  "Returns md5sum for a message object of type '<frame_switch-response>"
  "50cdb9fa30e06e199809b1d13df3e46b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'frame_switch-response)))
  "Returns md5sum for a message object of type 'frame_switch-response"
  "50cdb9fa30e06e199809b1d13df3e46b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<frame_switch-response>)))
  "Returns full string definition for message of type '<frame_switch-response>"
  (cl:format cl:nil "#Response data~%string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'frame_switch-response)))
  "Returns full string definition for message of type 'frame_switch-response"
  (cl:format cl:nil "#Response data~%string result # for mission result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <frame_switch-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <frame_switch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'frame_switch-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'frame_switch)))
  'frame_switch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'frame_switch)))
  'frame_switch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'frame_switch)))
  "Returns string type for a service object of type '<frame_switch>"
  "subt_msgs/frame_switch")