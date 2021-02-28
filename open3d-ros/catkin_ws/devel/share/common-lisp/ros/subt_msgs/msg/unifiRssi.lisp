; Auto-generated. Do not edit!


(cl:in-package subt_msgs-msg)


;//! \htmlinclude unifiRssi.msg.html

(cl:defclass <unifiRssi> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (client_name
    :reader client_name
    :initarg :client_name
    :type cl:string
    :initform "")
   (connect_ap_name
    :reader connect_ap_name
    :initarg :connect_ap_name
    :type cl:string
    :initform "")
   (client_rssi
    :reader client_rssi
    :initarg :client_rssi
    :type cl:fixnum
    :initform 0)
   (ap_name
    :reader ap_name
    :initarg :ap_name
    :type cl:string
    :initform "")
   (ap_rssi
    :reader ap_rssi
    :initarg :ap_rssi
    :type cl:fixnum
    :initform 0))
)

(cl:defclass unifiRssi (<unifiRssi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <unifiRssi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'unifiRssi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-msg:<unifiRssi> is deprecated: use subt_msgs-msg:unifiRssi instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <unifiRssi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:header-val is deprecated.  Use subt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'client_name-val :lambda-list '(m))
(cl:defmethod client_name-val ((m <unifiRssi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:client_name-val is deprecated.  Use subt_msgs-msg:client_name instead.")
  (client_name m))

(cl:ensure-generic-function 'connect_ap_name-val :lambda-list '(m))
(cl:defmethod connect_ap_name-val ((m <unifiRssi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:connect_ap_name-val is deprecated.  Use subt_msgs-msg:connect_ap_name instead.")
  (connect_ap_name m))

(cl:ensure-generic-function 'client_rssi-val :lambda-list '(m))
(cl:defmethod client_rssi-val ((m <unifiRssi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:client_rssi-val is deprecated.  Use subt_msgs-msg:client_rssi instead.")
  (client_rssi m))

(cl:ensure-generic-function 'ap_name-val :lambda-list '(m))
(cl:defmethod ap_name-val ((m <unifiRssi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:ap_name-val is deprecated.  Use subt_msgs-msg:ap_name instead.")
  (ap_name m))

(cl:ensure-generic-function 'ap_rssi-val :lambda-list '(m))
(cl:defmethod ap_rssi-val ((m <unifiRssi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:ap_rssi-val is deprecated.  Use subt_msgs-msg:ap_rssi instead.")
  (ap_rssi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <unifiRssi>) ostream)
  "Serializes a message object of type '<unifiRssi>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'client_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'client_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'connect_ap_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'connect_ap_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'client_rssi)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'client_rssi)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ap_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ap_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ap_rssi)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ap_rssi)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <unifiRssi>) istream)
  "Deserializes a message object of type '<unifiRssi>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'client_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'client_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'connect_ap_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'connect_ap_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'client_rssi)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'client_rssi)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ap_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ap_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ap_rssi)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ap_rssi)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<unifiRssi>)))
  "Returns string type for a message object of type '<unifiRssi>"
  "subt_msgs/unifiRssi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'unifiRssi)))
  "Returns string type for a message object of type 'unifiRssi"
  "subt_msgs/unifiRssi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<unifiRssi>)))
  "Returns md5sum for a message object of type '<unifiRssi>"
  "9df6040c2bb377a5a4138aa41245a942")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'unifiRssi)))
  "Returns md5sum for a message object of type 'unifiRssi"
  "9df6040c2bb377a5a4138aa41245a942")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<unifiRssi>)))
  "Returns full string definition for message of type '<unifiRssi>"
  (cl:format cl:nil "Header header~%~%string client_name~%string connect_ap_name~%uint16 client_rssi~%string ap_name~%uint16 ap_rssi~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'unifiRssi)))
  "Returns full string definition for message of type 'unifiRssi"
  (cl:format cl:nil "Header header~%~%string client_name~%string connect_ap_name~%uint16 client_rssi~%string ap_name~%uint16 ap_rssi~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <unifiRssi>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'client_name))
     4 (cl:length (cl:slot-value msg 'connect_ap_name))
     2
     4 (cl:length (cl:slot-value msg 'ap_name))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <unifiRssi>))
  "Converts a ROS message object to a list"
  (cl:list 'unifiRssi
    (cl:cons ':header (header msg))
    (cl:cons ':client_name (client_name msg))
    (cl:cons ':connect_ap_name (connect_ap_name msg))
    (cl:cons ':client_rssi (client_rssi msg))
    (cl:cons ':ap_name (ap_name msg))
    (cl:cons ':ap_rssi (ap_rssi msg))
))
