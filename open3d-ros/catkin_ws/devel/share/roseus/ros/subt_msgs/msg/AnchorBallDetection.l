;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::AnchorBallDetection)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'AnchorBallDetection (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::ANCHORBALLDETECTION")
  (make-package "SUBT_MSGS::ANCHORBALLDETECTION"))

(in-package "ROS")
;;//! \htmlinclude AnchorBallDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass subt_msgs::AnchorBallDetection
  :super ros::object
  :slots (_stamp _robot_id _anchorball_id _pose ))

(defmethod subt_msgs::AnchorBallDetection
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:robot_id __robot_id) 0)
    ((:anchorball_id __anchorball_id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _robot_id (round __robot_id))
   (setq _anchorball_id (round __anchorball_id))
   (setq _pose __pose)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:anchorball_id
   (&optional __anchorball_id)
   (if __anchorball_id (setq _anchorball_id __anchorball_id)) _anchorball_id)
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
    ;; time _stamp
    8
    ;; int32 _robot_id
    4
    ;; int32 _anchorball_id
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _robot_id
       (write-long _robot_id s)
     ;; int32 _anchorball_id
       (write-long _anchorball_id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _robot_id
     (setq _robot_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _anchorball_id
     (setq _anchorball_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get subt_msgs::AnchorBallDetection :md5sum-) "26c4aa21e931afcad781619924ee252c")
(setf (get subt_msgs::AnchorBallDetection :datatype-) "subt_msgs/AnchorBallDetection")
(setf (get subt_msgs::AnchorBallDetection :definition-)
      "time stamp
int32 robot_id
int32 anchorball_id
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



(provide :subt_msgs/AnchorBallDetection "26c4aa21e931afcad781619924ee252c")


