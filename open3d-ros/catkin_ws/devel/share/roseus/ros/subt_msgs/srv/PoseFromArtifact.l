;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::PoseFromArtifact)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'PoseFromArtifact (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::POSEFROMARTIFACT")
  (make-package "SUBT_MSGS::POSEFROMARTIFACT"))
(unless (find-package "SUBT_MSGS::POSEFROMARTIFACTREQUEST")
  (make-package "SUBT_MSGS::POSEFROMARTIFACTREQUEST"))
(unless (find-package "SUBT_MSGS::POSEFROMARTIFACTRESPONSE")
  (make-package "SUBT_MSGS::POSEFROMARTIFACTRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass subt_msgs::PoseFromArtifactRequest
  :super ros::object
  :slots (_robot_name ))

(defmethod subt_msgs::PoseFromArtifactRequest
  (:init
   (&key
    ((:robot_name __robot_name) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _robot_name __robot_name)
   self)
  (:robot_name
   (&rest __robot_name)
   (if (keywordp (car __robot_name))
       (send* _robot_name __robot_name)
     (progn
       (if __robot_name (setq _robot_name (car __robot_name)))
       _robot_name)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _robot_name
    (send _robot_name :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _robot_name
       (send _robot_name :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _robot_name
     (send _robot_name :deserialize buf ptr-) (incf ptr- (send _robot_name :serialization-length))
   ;;
   self)
  )

(defclass subt_msgs::PoseFromArtifactResponse
  :super ros::object
  :slots (_pose _success ))

(defmethod subt_msgs::PoseFromArtifactResponse
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _success __success)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass subt_msgs::PoseFromArtifact
  :super ros::object
  :slots ())

(setf (get subt_msgs::PoseFromArtifact :md5sum-) "acaef0f7bc2569bf46eb58f8f9678144")
(setf (get subt_msgs::PoseFromArtifact :datatype-) "subt_msgs/PoseFromArtifact")
(setf (get subt_msgs::PoseFromArtifact :request) subt_msgs::PoseFromArtifactRequest)
(setf (get subt_msgs::PoseFromArtifact :response) subt_msgs::PoseFromArtifactResponse)

(defmethod subt_msgs::PoseFromArtifactRequest
  (:response () (instance subt_msgs::PoseFromArtifactResponse :init)))

(setf (get subt_msgs::PoseFromArtifactRequest :md5sum-) "acaef0f7bc2569bf46eb58f8f9678144")
(setf (get subt_msgs::PoseFromArtifactRequest :datatype-) "subt_msgs/PoseFromArtifactRequest")
(setf (get subt_msgs::PoseFromArtifactRequest :definition-)
      "# PoseFromArtifact

std_msgs/String robot_name

================================================================================
MSG: std_msgs/String
string data
---
geometry_msgs/PoseStamped pose
bool success

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get subt_msgs::PoseFromArtifactResponse :md5sum-) "acaef0f7bc2569bf46eb58f8f9678144")
(setf (get subt_msgs::PoseFromArtifactResponse :datatype-) "subt_msgs/PoseFromArtifactResponse")
(setf (get subt_msgs::PoseFromArtifactResponse :definition-)
      "# PoseFromArtifact

std_msgs/String robot_name

================================================================================
MSG: std_msgs/String
string data
---
geometry_msgs/PoseStamped pose
bool success

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :subt_msgs/PoseFromArtifact "acaef0f7bc2569bf46eb58f8f9678144")


