;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::ArtifactPoseArray)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'ArtifactPoseArray (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::ARTIFACTPOSEARRAY")
  (make-package "SUBT_MSGS::ARTIFACTPOSEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ArtifactPoseArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass subt_msgs::ArtifactPoseArray
  :super ros::object
  :slots (_header _camera _count _pose_array ))

(defmethod subt_msgs::ArtifactPoseArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera __camera) "")
    ((:count __count) 0)
    ((:pose_array __pose_array) (let (r) (dotimes (i 0) (push (instance subt_msgs::ArtifactPose :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera (string __camera))
   (setq _count (round __count))
   (setq _pose_array __pose_array)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:camera
   (&optional __camera)
   (if __camera (setq _camera __camera)) _camera)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:pose_array
   (&rest __pose_array)
   (if (keywordp (car __pose_array))
       (send* _pose_array __pose_array)
     (progn
       (if __pose_array (setq _pose_array (car __pose_array)))
       _pose_array)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _camera
    4 (length _camera)
    ;; int32 _count
    4
    ;; subt_msgs/ArtifactPose[] _pose_array
    (apply #'+ (send-all _pose_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _camera
       (write-long (length _camera) s) (princ _camera s)
     ;; int32 _count
       (write-long _count s)
     ;; subt_msgs/ArtifactPose[] _pose_array
     (write-long (length _pose_array) s)
     (dolist (elem _pose_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _camera
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; subt_msgs/ArtifactPose[] _pose_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose_array (let (r) (dotimes (i n) (push (instance subt_msgs::ArtifactPose :init) r)) r))
     (dolist (elem- _pose_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get subt_msgs::ArtifactPoseArray :md5sum-) "1241fdc583a1d4a260d1fbeb9ee080f0")
(setf (get subt_msgs::ArtifactPoseArray :datatype-) "subt_msgs/ArtifactPoseArray")
(setf (get subt_msgs::ArtifactPoseArray :definition-)
      "Header header
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



(provide :subt_msgs/ArtifactPoseArray "1241fdc583a1d4a260d1fbeb9ee080f0")


