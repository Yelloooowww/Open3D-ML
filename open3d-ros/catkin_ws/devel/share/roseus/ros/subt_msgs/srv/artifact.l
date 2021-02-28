;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::artifact)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'artifact (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::ARTIFACT")
  (make-package "SUBT_MSGS::ARTIFACT"))
(unless (find-package "SUBT_MSGS::ARTIFACTREQUEST")
  (make-package "SUBT_MSGS::ARTIFACTREQUEST"))
(unless (find-package "SUBT_MSGS::ARTIFACTRESPONSE")
  (make-package "SUBT_MSGS::ARTIFACTRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::artifactRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::artifactRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass subt_msgs::artifactResponse
  :super ros::object
  :slots (_artifacts ))

(defmethod subt_msgs::artifactResponse
  (:init
   (&key
    ((:artifacts __artifacts) (instance subt_msgs::ArtifactPoseArray :init))
    )
   (send-super :init)
   (setq _artifacts __artifacts)
   self)
  (:artifacts
   (&rest __artifacts)
   (if (keywordp (car __artifacts))
       (send* _artifacts __artifacts)
     (progn
       (if __artifacts (setq _artifacts (car __artifacts)))
       _artifacts)))
  (:serialization-length
   ()
   (+
    ;; subt_msgs/ArtifactPoseArray _artifacts
    (send _artifacts :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; subt_msgs/ArtifactPoseArray _artifacts
       (send _artifacts :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; subt_msgs/ArtifactPoseArray _artifacts
     (send _artifacts :deserialize buf ptr-) (incf ptr- (send _artifacts :serialization-length))
   ;;
   self)
  )

(defclass subt_msgs::artifact
  :super ros::object
  :slots ())

(setf (get subt_msgs::artifact :md5sum-) "69bcd85d6d5bff1b077f8d902e21fd09")
(setf (get subt_msgs::artifact :datatype-) "subt_msgs/artifact")
(setf (get subt_msgs::artifact :request) subt_msgs::artifactRequest)
(setf (get subt_msgs::artifact :response) subt_msgs::artifactResponse)

(defmethod subt_msgs::artifactRequest
  (:response () (instance subt_msgs::artifactResponse :init)))

(setf (get subt_msgs::artifactRequest :md5sum-) "69bcd85d6d5bff1b077f8d902e21fd09")
(setf (get subt_msgs::artifactRequest :datatype-) "subt_msgs/artifactRequest")
(setf (get subt_msgs::artifactRequest :definition-)
      "#Request data
---
#Response data
ArtifactPoseArray artifacts


================================================================================
MSG: subt_msgs/ArtifactPoseArray
Header header
string camera
int32 count
ArtifactPose[] pose_array
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
MSG: subt_msgs/ArtifactPose
string Class
#string status				# O for non_return , X for return 
int32 appear_count
float64 probability
geometry_msgs/Pose pose
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

(setf (get subt_msgs::artifactResponse :md5sum-) "69bcd85d6d5bff1b077f8d902e21fd09")
(setf (get subt_msgs::artifactResponse :datatype-) "subt_msgs/artifactResponse")
(setf (get subt_msgs::artifactResponse :definition-)
      "#Request data
---
#Response data
ArtifactPoseArray artifacts


================================================================================
MSG: subt_msgs/ArtifactPoseArray
Header header
string camera
int32 count
ArtifactPose[] pose_array
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
MSG: subt_msgs/ArtifactPose
string Class
#string status				# O for non_return , X for return 
int32 appear_count
float64 probability
geometry_msgs/Pose pose
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



(provide :subt_msgs/artifact "69bcd85d6d5bff1b077f8d902e21fd09")


