; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude report-request.msg.html

(cl:defclass <report-request> (roslisp-msg-protocol:ros-message)
  ((artifact
    :reader artifact
    :initarg :artifact
    :type subt_msgs-msg:ArtifactPose
    :initform (cl:make-instance 'subt_msgs-msg:ArtifactPose)))
)

(cl:defclass report-request (<report-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <report-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'report-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<report-request> is deprecated: use subt_msgs-srv:report-request instead.")))

(cl:ensure-generic-function 'artifact-val :lambda-list '(m))
(cl:defmethod artifact-val ((m <report-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:artifact-val is deprecated.  Use subt_msgs-srv:artifact instead.")
  (artifact m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <report-request>) ostream)
  "Serializes a message object of type '<report-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'artifact) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <report-request>) istream)
  "Deserializes a message object of type '<report-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'artifact) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<report-request>)))
  "Returns string type for a service object of type '<report-request>"
  "subt_msgs/reportRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'report-request)))
  "Returns string type for a service object of type 'report-request"
  "subt_msgs/reportRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<report-request>)))
  "Returns md5sum for a message object of type '<report-request>"
  "d571fa3f7149c57374206fa01faa05ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'report-request)))
  "Returns md5sum for a message object of type 'report-request"
  "d571fa3f7149c57374206fa01faa05ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<report-request>)))
  "Returns full string definition for message of type '<report-request>"
  (cl:format cl:nil "#Request data~%ArtifactPose artifact~%~%================================================================================~%MSG: subt_msgs/ArtifactPose~%string Class~%#string status				# O for non_return , X for return ~%int32 appear_count~%float64 probability~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'report-request)))
  "Returns full string definition for message of type 'report-request"
  (cl:format cl:nil "#Request data~%ArtifactPose artifact~%~%================================================================================~%MSG: subt_msgs/ArtifactPose~%string Class~%#string status				# O for non_return , X for return ~%int32 appear_count~%float64 probability~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <report-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'artifact))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <report-request>))
  "Converts a ROS message object to a list"
  (cl:list 'report-request
    (cl:cons ':artifact (artifact msg))
))
;//! \htmlinclude report-response.msg.html

(cl:defclass <report-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass report-response (<report-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <report-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'report-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<report-response> is deprecated: use subt_msgs-srv:report-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <report-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:result-val is deprecated.  Use subt_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <report-response>) ostream)
  "Serializes a message object of type '<report-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <report-response>) istream)
  "Deserializes a message object of type '<report-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<report-response>)))
  "Returns string type for a service object of type '<report-response>"
  "subt_msgs/reportResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'report-response)))
  "Returns string type for a service object of type 'report-response"
  "subt_msgs/reportResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<report-response>)))
  "Returns md5sum for a message object of type '<report-response>"
  "d571fa3f7149c57374206fa01faa05ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'report-response)))
  "Returns md5sum for a message object of type 'report-response"
  "d571fa3f7149c57374206fa01faa05ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<report-response>)))
  "Returns full string definition for message of type '<report-response>"
  (cl:format cl:nil "#Response data~%string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'report-response)))
  "Returns full string definition for message of type 'report-response"
  (cl:format cl:nil "#Response data~%string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <report-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <report-response>))
  "Converts a ROS message object to a list"
  (cl:list 'report-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'report)))
  'report-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'report)))
  'report-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'report)))
  "Returns string type for a service object of type '<report>"
  "subt_msgs/report")