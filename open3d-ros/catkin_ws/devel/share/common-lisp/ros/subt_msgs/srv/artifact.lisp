; Auto-generated. Do not edit!


(cl:in-package subt_msgs-srv)


;//! \htmlinclude artifact-request.msg.html

(cl:defclass <artifact-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass artifact-request (<artifact-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <artifact-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'artifact-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<artifact-request> is deprecated: use subt_msgs-srv:artifact-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <artifact-request>) ostream)
  "Serializes a message object of type '<artifact-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <artifact-request>) istream)
  "Deserializes a message object of type '<artifact-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<artifact-request>)))
  "Returns string type for a service object of type '<artifact-request>"
  "subt_msgs/artifactRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'artifact-request)))
  "Returns string type for a service object of type 'artifact-request"
  "subt_msgs/artifactRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<artifact-request>)))
  "Returns md5sum for a message object of type '<artifact-request>"
  "69bcd85d6d5bff1b077f8d902e21fd09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'artifact-request)))
  "Returns md5sum for a message object of type 'artifact-request"
  "69bcd85d6d5bff1b077f8d902e21fd09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<artifact-request>)))
  "Returns full string definition for message of type '<artifact-request>"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'artifact-request)))
  "Returns full string definition for message of type 'artifact-request"
  (cl:format cl:nil "#Request data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <artifact-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <artifact-request>))
  "Converts a ROS message object to a list"
  (cl:list 'artifact-request
))
;//! \htmlinclude artifact-response.msg.html

(cl:defclass <artifact-response> (roslisp-msg-protocol:ros-message)
  ((artifacts
    :reader artifacts
    :initarg :artifacts
    :type subt_msgs-msg:ArtifactPoseArray
    :initform (cl:make-instance 'subt_msgs-msg:ArtifactPoseArray)))
)

(cl:defclass artifact-response (<artifact-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <artifact-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'artifact-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-srv:<artifact-response> is deprecated: use subt_msgs-srv:artifact-response instead.")))

(cl:ensure-generic-function 'artifacts-val :lambda-list '(m))
(cl:defmethod artifacts-val ((m <artifact-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-srv:artifacts-val is deprecated.  Use subt_msgs-srv:artifacts instead.")
  (artifacts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <artifact-response>) ostream)
  "Serializes a message object of type '<artifact-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'artifacts) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <artifact-response>) istream)
  "Deserializes a message object of type '<artifact-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'artifacts) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<artifact-response>)))
  "Returns string type for a service object of type '<artifact-response>"
  "subt_msgs/artifactResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'artifact-response)))
  "Returns string type for a service object of type 'artifact-response"
  "subt_msgs/artifactResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<artifact-response>)))
  "Returns md5sum for a message object of type '<artifact-response>"
  "69bcd85d6d5bff1b077f8d902e21fd09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'artifact-response)))
  "Returns md5sum for a message object of type 'artifact-response"
  "69bcd85d6d5bff1b077f8d902e21fd09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<artifact-response>)))
  "Returns full string definition for message of type '<artifact-response>"
  (cl:format cl:nil "#Response data~%ArtifactPoseArray artifacts~%~%~%================================================================================~%MSG: subt_msgs/ArtifactPoseArray~%Header header~%string camera~%int32 count~%ArtifactPose[] pose_array~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: subt_msgs/ArtifactPose~%string Class~%#string status				# O for non_return , X for return ~%int32 appear_count~%float64 probability~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'artifact-response)))
  "Returns full string definition for message of type 'artifact-response"
  (cl:format cl:nil "#Response data~%ArtifactPoseArray artifacts~%~%~%================================================================================~%MSG: subt_msgs/ArtifactPoseArray~%Header header~%string camera~%int32 count~%ArtifactPose[] pose_array~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: subt_msgs/ArtifactPose~%string Class~%#string status				# O for non_return , X for return ~%int32 appear_count~%float64 probability~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <artifact-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'artifacts))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <artifact-response>))
  "Converts a ROS message object to a list"
  (cl:list 'artifact-response
    (cl:cons ':artifacts (artifacts msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'artifact)))
  'artifact-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'artifact)))
  'artifact-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'artifact)))
  "Returns string type for a service object of type '<artifact>"
  "subt_msgs/artifact")