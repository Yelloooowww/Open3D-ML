; Auto-generated. Do not edit!


(cl:in-package subt_msgs-msg)


;//! \htmlinclude SubTInfo.msg.html

(cl:defclass <SubTInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform "")
   (robot_pose
    :reader robot_pose
    :initarg :robot_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (artifacts
    :reader artifacts
    :initarg :artifacts
    :type subt_msgs-msg:ArtifactPoseArray
    :initform (cl:make-instance 'subt_msgs-msg:ArtifactPoseArray)))
)

(cl:defclass SubTInfo (<SubTInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubTInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubTInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-msg:<SubTInfo> is deprecated: use subt_msgs-msg:SubTInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SubTInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:header-val is deprecated.  Use subt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <SubTInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:robot_name-val is deprecated.  Use subt_msgs-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'robot_pose-val :lambda-list '(m))
(cl:defmethod robot_pose-val ((m <SubTInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:robot_pose-val is deprecated.  Use subt_msgs-msg:robot_pose instead.")
  (robot_pose m))

(cl:ensure-generic-function 'artifacts-val :lambda-list '(m))
(cl:defmethod artifacts-val ((m <SubTInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:artifacts-val is deprecated.  Use subt_msgs-msg:artifacts instead.")
  (artifacts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubTInfo>) ostream)
  "Serializes a message object of type '<SubTInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'artifacts) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubTInfo>) istream)
  "Deserializes a message object of type '<SubTInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'artifacts) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubTInfo>)))
  "Returns string type for a message object of type '<SubTInfo>"
  "subt_msgs/SubTInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubTInfo)))
  "Returns string type for a message object of type 'SubTInfo"
  "subt_msgs/SubTInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubTInfo>)))
  "Returns md5sum for a message object of type '<SubTInfo>"
  "e821f52944c112ea518ab90db2ed050a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubTInfo)))
  "Returns md5sum for a message object of type 'SubTInfo"
  "e821f52944c112ea518ab90db2ed050a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubTInfo>)))
  "Returns full string definition for message of type '<SubTInfo>"
  (cl:format cl:nil "Header header~%string robot_name~%geometry_msgs/Pose robot_pose~%ArtifactPoseArray artifacts~%#AnchorBallDetection anchorballs~%#bool has_ball~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: subt_msgs/ArtifactPoseArray~%Header header~%string camera~%int32 count~%ArtifactPose[] pose_array~%================================================================================~%MSG: subt_msgs/ArtifactPose~%string Class~%#string status				# O for non_return , X for return ~%int32 appear_count~%float64 probability~%geometry_msgs/Pose pose~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubTInfo)))
  "Returns full string definition for message of type 'SubTInfo"
  (cl:format cl:nil "Header header~%string robot_name~%geometry_msgs/Pose robot_pose~%ArtifactPoseArray artifacts~%#AnchorBallDetection anchorballs~%#bool has_ball~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: subt_msgs/ArtifactPoseArray~%Header header~%string camera~%int32 count~%ArtifactPose[] pose_array~%================================================================================~%MSG: subt_msgs/ArtifactPose~%string Class~%#string status				# O for non_return , X for return ~%int32 appear_count~%float64 probability~%geometry_msgs/Pose pose~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubTInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'artifacts))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubTInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SubTInfo
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':robot_pose (robot_pose msg))
    (cl:cons ':artifacts (artifacts msg))
))
