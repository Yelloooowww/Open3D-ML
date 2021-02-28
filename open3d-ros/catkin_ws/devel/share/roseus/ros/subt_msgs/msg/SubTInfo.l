;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::SubTInfo)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'SubTInfo (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::SUBTINFO")
  (make-package "SUBT_MSGS::SUBTINFO"))

(in-package "ROS")
;;//! \htmlinclude SubTInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass subt_msgs::SubTInfo
  :super ros::object
  :slots (_header _robot_name _robot_pose _artifacts ))

(defmethod subt_msgs::SubTInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:robot_pose __robot_pose) (instance geometry_msgs::Pose :init))
    ((:artifacts __artifacts) (instance subt_msgs::ArtifactPoseArray :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _robot_pose __robot_pose)
   (setq _artifacts __artifacts)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:robot_pose
   (&rest __robot_pose)
   (if (keywordp (car __robot_pose))
       (send* _robot_pose __robot_pose)
     (progn
       (if __robot_pose (setq _robot_pose (car __robot_pose)))
       _robot_pose)))
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
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; geometry_msgs/Pose _robot_pose
    (send _robot_pose :serialization-length)
    ;; subt_msgs/ArtifactPoseArray _artifacts
    (send _artifacts :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; geometry_msgs/Pose _robot_pose
       (send _robot_pose :serialize s)
     ;; subt_msgs/ArtifactPoseArray _artifacts
       (send _artifacts :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _robot_pose
     (send _robot_pose :deserialize buf ptr-) (incf ptr- (send _robot_pose :serialization-length))
   ;; subt_msgs/ArtifactPoseArray _artifacts
     (send _artifacts :deserialize buf ptr-) (incf ptr- (send _artifacts :serialization-length))
   ;;
   self)
  )

(setf (get subt_msgs::SubTInfo :md5sum-) "e821f52944c112ea518ab90db2ed050a")
(setf (get subt_msgs::SubTInfo :datatype-) "subt_msgs/SubTInfo")
(setf (get subt_msgs::SubTInfo :definition-)
      "Header header
string robot_name
geometry_msgs/Pose robot_pose
ArtifactPoseArray artifacts
#AnchorBallDetection anchorballs
#bool has_ball
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

================================================================================
MSG: subt_msgs/ArtifactPoseArray
Header header
string camera
int32 count
ArtifactPose[] pose_array
================================================================================
MSG: subt_msgs/ArtifactPose
string Class
#string status				# O for non_return , X for return 
int32 appear_count
float64 probability
geometry_msgs/Pose pose
")



(provide :subt_msgs/SubTInfo "e821f52944c112ea518ab90db2ed050a")


