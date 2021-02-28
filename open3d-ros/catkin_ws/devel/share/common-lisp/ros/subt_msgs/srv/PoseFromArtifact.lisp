; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude PoseFromArtifact-request.msg.html

(cl:defclass <PoseFromArtifact-request> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass PoseFromArtifact-request (<PoseFromArtifact-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseFromArtifact-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseFromArtifact-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<PoseFromArtifact-request> is deprecated: use subt_msgs-srv:PoseFromArtifact-request instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <PoseFromArtifact-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:robot_name-val is deprecated.  Use subt_msgs-srv:robot_name instead.")
  (robot_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseFromArtifact-request>) ostream)
  "Serializes a message object of type '<PoseFromArtifact-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_name) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseFromArtifact-request>) istream)
  "Deserializes a message object of type '<PoseFromArtifact-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_name) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseFromArtifact-request>)))
  "Returns string type for a service object of type '<PoseFromArtifact-request>"
  "subt_msgs/PoseFromArtifactRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseFromArtifact-request)))
  "Returns string type for a service object of type 'PoseFromArtifact-request"
  "subt_msgs/PoseFromArtifactRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseFromArtifact-request>)))
  "Returns md5sum for a message object of type '<PoseFromArtifact-request>"
  "acaef0f7bc2569bf46eb58f8f9678144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseFromArtifact-request)))
  "Returns md5sum for a message object of type 'PoseFromArtifact-request"
  "acaef0f7bc2569bf46eb58f8f9678144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseFromArtifact-request>)))
  "Returns full string definition for message of type '<PoseFromArtifact-request>"
  (cl:format cl:nil "# PoseFromArtifact~%~%std_msgs/String robot_name~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseFromArtifact-request)))
  "Returns full string definition for message of type 'PoseFromArtifact-request"
  (cl:format cl:nil "# PoseFromArtifact~%~%std_msgs/String robot_name~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseFromArtifact-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseFromArtifact-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseFromArtifact-request
    (cl:cons ':robot_name (robot_name msg))
))
;//! \htmlinclude PoseFromArtifact-response.msg.html

(cl:defclass <PoseFromArtifact-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PoseFromArtifact-response (<PoseFromArtifact-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseFromArtifact-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseFromArtifact-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<PoseFromArtifact-response> is deprecated: use subt_msgs-srv:PoseFromArtifact-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <PoseFromArtifact-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:pose-val is deprecated.  Use subt_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PoseFromArtifact-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:success-val is deprecated.  Use subt_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseFromArtifact-response>) ostream)
  "Serializes a message object of type '<PoseFromArtifact-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseFromArtifact-response>) istream)
  "Deserializes a message object of type '<PoseFromArtifact-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseFromArtifact-response>)))
  "Returns string type for a service object of type '<PoseFromArtifact-response>"
  "subt_msgs/PoseFromArtifactResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseFromArtifact-response)))
  "Returns string type for a service object of type 'PoseFromArtifact-response"
  "subt_msgs/PoseFromArtifactResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseFromArtifact-response>)))
  "Returns md5sum for a message object of type '<PoseFromArtifact-response>"
  "acaef0f7bc2569bf46eb58f8f9678144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseFromArtifact-response)))
  "Returns md5sum for a message object of type 'PoseFromArtifact-response"
  "acaef0f7bc2569bf46eb58f8f9678144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseFromArtifact-response>)))
  "Returns full string definition for message of type '<PoseFromArtifact-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped pose~%bool success~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseFromArtifact-response)))
  "Returns full string definition for message of type 'PoseFromArtifact-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped pose~%bool success~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseFromArtifact-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseFromArtifact-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseFromArtifact-response
    (cl:cons ':pose (pose msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PoseFromArtifact)))
  'PoseFromArtifact-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PoseFromArtifact)))
  'PoseFromArtifact-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseFromArtifact)))
  "Returns string type for a service object of type '<PoseFromArtifact>"
  "subt_msgs/PoseFromArtifact")