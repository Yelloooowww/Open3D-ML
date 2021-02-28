;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::ArtifactPose)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'ArtifactPose (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::ARTIFACTPOSE")
  (make-package "SUBT_MSGS::ARTIFACTPOSE"))

(in-package "ROS")
;;//! \htmlinclude ArtifactPose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass subt_msgs::ArtifactPose
  :super ros::object
  :slots (_Class _appear_count _probability _pose ))

(defmethod subt_msgs::ArtifactPose
  (:init
   (&key
    ((:Class __Class) "")
    ((:appear_count __appear_count) 0)
    ((:probability __probability) 0.0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _Class (string __Class))
   (setq _appear_count (round __appear_count))
   (setq _probability (float __probability))
   (setq _pose __pose)
   self)
  (:Class
   (&optional __Class)
   (if __Class (setq _Class __Class)) _Class)
  (:appear_count
   (&optional __appear_count)
   (if __appear_count (setq _appear_count __appear_count)) _appear_count)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; string _Class
    4 (length _Class)
    ;; int32 _appear_count
    4
    ;; float64 _probability
    8
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Class
       (write-long (length _Class) s) (princ _Class s)
     ;; int32 _appear_count
       (write-long _appear_count s)
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _appear_count
     (setq _appear_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get subt_msgs::ArtifactPose :md5sum-) "cf3928d80a114228d5528e7f48e5c6d2")
(setf (get subt_msgs::ArtifactPose :datatype-) "subt_msgs/ArtifactPose")
(setf (get subt_msgs::ArtifactPose :definition-)
      "string Class
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



(provide :subt_msgs/ArtifactPose "cf3928d80a114228d5528e7f48e5c6d2")


