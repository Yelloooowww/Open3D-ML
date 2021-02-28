; Auto-generated. Do not edit!


(cl:in-package subt_msgs-msg)


;//! \htmlinclude GloraPack.msg.html

(cl:defclass <GloraPack> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (packet
    :reader packet
    :initarg :packet
    :type cl:string
    :initform "")
   (time_elapse
    :reader time_elapse
    :initarg :time_elapse
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GloraPack (<GloraPack>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GloraPack>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GloraPack)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-msg:<GloraPack> is deprecated: use subt_msgs-msg:GloraPack instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GloraPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:header-val is deprecated.  Use subt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'packet-val :lambda-list '(m))
(cl:defmethod packet-val ((m <GloraPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:packet-val is deprecated.  Use subt_msgs-msg:packet instead.")
  (packet m))

(cl:ensure-generic-function 'time_elapse-val :lambda-list '(m))
(cl:defmethod time_elapse-val ((m <GloraPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:time_elapse-val is deprecated.  Use subt_msgs-msg:time_elapse instead.")
  (time_elapse m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GloraPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:success-val is deprecated.  Use subt_msgs-msg:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GloraPack>) ostream)
  "Serializes a message object of type '<GloraPack>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'packet))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'packet))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time_elapse))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GloraPack>) istream)
  "Deserializes a message object of type '<GloraPack>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'packet) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'packet) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_elapse) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GloraPack>)))
  "Returns string type for a message object of type '<GloraPack>"
  "subt_msgs/GloraPack")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GloraPack)))
  "Returns string type for a message object of type 'GloraPack"
  "subt_msgs/GloraPack")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GloraPack>)))
  "Returns md5sum for a message object of type '<GloraPack>"
  "793b4359128b7f4b7385d49cb96eaaec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GloraPack)))
  "Returns md5sum for a message object of type 'GloraPack"
  "793b4359128b7f4b7385d49cb96eaaec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GloraPack>)))
  "Returns full string definition for message of type '<GloraPack>"
  (cl:format cl:nil "Header header~%string packet~%float32 time_elapse ~%bool success~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GloraPack)))
  "Returns full string definition for message of type 'GloraPack"
  (cl:format cl:nil "Header header~%string packet~%float32 time_elapse ~%bool success~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GloraPack>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'packet))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GloraPack>))
  "Converts a ROS message object to a list"
  (cl:list 'GloraPack
    (cl:cons ':header (header msg))
    (cl:cons ':packet (packet msg))
    (cl:cons ':time_elapse (time_elapse msg))
    (cl:cons ':success (success msg))
))
