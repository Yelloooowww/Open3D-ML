;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::report)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'report (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::REPORT")
  (make-package "SUBT_MSGS::REPORT"))
(unless (find-package "SUBT_MSGS::REPORTREQUEST")
  (make-package "SUBT_MSGS::REPORTREQUEST"))
(unless (find-package "SUBT_MSGS::REPORTRESPONSE")
  (make-package "SUBT_MSGS::REPORTRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::reportRequest
  :super ros::object
  :slots (_artifact ))

(defmethod subt_msgs::reportRequest
  (:init
   (&key
    ((:artifact __artifact) (instance subt_msgs::ArtifactPose :init))
    )
   (send-super :init)
   (setq _artifact __artifact)
   self)
  (:artifact
   (&rest __artifact)
   (if (keywordp (car __artifact))
       (send* _artifact __artifact)
     (progn
       (if __artifact (setq _artifact (car __artifact)))
       _artifact)))
  (:serialization-length
   ()
   (+
    ;; subt_msgs/ArtifactPose _artifact
    (send _artifact :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; subt_msgs/ArtifactPose _artifact
       (send _artifact :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; subt_msgs/ArtifactPose _artifact
     (send _artifact :deserialize buf ptr-) (incf ptr- (send _artifact :serialization-length))
   ;;
   self)
  )

(defclass subt_msgs::reportResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::reportResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass subt_msgs::report
  :super ros::object
  :slots ())

(setf (get subt_msgs::report :md5sum-) "d571fa3f7149c57374206fa01faa05ff")
(setf (get subt_msgs::report :datatype-) "subt_msgs/report")
(setf (get subt_msgs::report :request) subt_msgs::reportRequest)
(setf (get subt_msgs::report :response) subt_msgs::reportResponse)

(defmethod subt_msgs::reportRequest
  (:response () (instance subt_msgs::reportResponse :init)))

(setf (get subt_msgs::reportRequest :md5sum-) "d571fa3f7149c57374206fa01faa05ff")
(setf (get subt_msgs::reportRequest :datatype-) "subt_msgs/reportRequest")
(setf (get subt_msgs::reportRequest :definition-)
      "#Request data
ArtifactPose artifact

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
---
#Response data
string result

")

(setf (get subt_msgs::reportResponse :md5sum-) "d571fa3f7149c57374206fa01faa05ff")
(setf (get subt_msgs::reportResponse :datatype-) "subt_msgs/reportResponse")
(setf (get subt_msgs::reportResponse :definition-)
      "#Request data
ArtifactPose artifact

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
---
#Response data
string result

")



(provide :subt_msgs/report "d571fa3f7149c57374206fa01faa05ff")


