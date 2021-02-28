; Auto-generated. Do not edit!


(cl:in-package subt_msgs-msg)


;//! \htmlinclude AnchorBallDetection.msg.html

(cl:defclass <AnchorBallDetection> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:integer
    :initform 0)
   (anchorball_id
    :reader anchorball_id
    :initarg :anchorball_id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass AnchorBallDetection (<AnchorBallDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnchorBallDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnchorBallDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subt_msgs-msg:<AnchorBallDetection> is deprecated: use subt_msgs-msg:AnchorBallDetection instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <AnchorBallDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:stamp-val is deprecated.  Use subt_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <AnchorBallDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:robot_id-val is deprecated.  Use subt_msgs-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'anchorball_id-val :lambda-list '(m))
(cl:defmethod anchorball_id-val ((m <AnchorBallDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:anchorball_id-val is deprecated.  Use subt_msgs-msg:anchorball_id instead.")
  (anchorball_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <AnchorBallDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subt_msgs-msg:pose-val is deprecated.  Use subt_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnchorBallDetection>) ostream)
  "Serializes a message object of type '<AnchorBallDetection>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'robot_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'anchorball_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnchorBallDetection>) istream)
  "Deserializes a message object of type '<AnchorBallDetection>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'anchorball_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnchorBallDetection>)))
  "Returns string type for a message object of type '<AnchorBallDetection>"
  "subt_msgs/AnchorBallDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnchorBallDetection)))
  "Returns string type for a message object of type 'AnchorBallDetection"
  "subt_msgs/AnchorBallDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnchorBallDetection>)))
  "Returns md5sum for a message object of type '<AnchorBallDetection>"
  "26c4aa21e931afcad781619924ee252c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnchorBallDetection)))
  "Returns md5sum for a message object of type 'AnchorBallDetection"
  "26c4aa21e931afcad781619924ee252c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnchorBallDetection>)))
  "Returns full string definition for message of type '<AnchorBallDetection>"
  (cl:format cl:nil "time stamp~%int32 robot_id~%int32 anchorball_id~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnchorBallDetection)))
  "Returns full string definition for message of type 'AnchorBallDetection"
  (cl:format cl:nil "time stamp~%int32 robot_id~%int32 anchorball_id~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnchorBallDetection>))
  (cl:+ 0
     8
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnchorBallDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'AnchorBallDetection
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':anchorball_id (anchorball_id msg))
    (cl:cons ':pose (pose msg))
))
