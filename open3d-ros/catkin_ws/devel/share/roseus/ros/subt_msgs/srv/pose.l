;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::pose)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'pose (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::POSE")
  (make-package "SUBT_MSGS::POSE"))
(unless (find-package "SUBT_MSGS::POSEREQUEST")
  (make-package "SUBT_MSGS::POSEREQUEST"))
(unless (find-package "SUBT_MSGS::POSERESPONSE")
  (make-package "SUBT_MSGS::POSERESPONSE"))

(in-package "ROS")





(defclass subt_msgs::poseRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::poseRequest
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

(defclass subt_msgs::poseResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::poseResponse
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

(defclass subt_msgs::pose
  :super ros::object
  :slots ())

(setf (get subt_msgs::pose :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::pose :datatype-) "subt_msgs/pose")
(setf (get subt_msgs::pose :request) subt_msgs::poseRequest)
(setf (get subt_msgs::pose :response) subt_msgs::poseResponse)

(defmethod subt_msgs::poseRequest
  (:response () (instance subt_msgs::poseResponse :init)))

(setf (get subt_msgs::poseRequest :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::poseRequest :datatype-) "subt_msgs/poseRequest")
(setf (get subt_msgs::poseRequest :definition-)
      "#Request data
---
#Response data
string result

")

(setf (get subt_msgs::poseResponse :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::poseResponse :datatype-) "subt_msgs/poseResponse")
(setf (get subt_msgs::poseResponse :definition-)
      "#Request data
---
#Response data
string result

")



(provide :subt_msgs/pose "c22f2a1ed8654a0b365f1bb3f7ff2c0f")


