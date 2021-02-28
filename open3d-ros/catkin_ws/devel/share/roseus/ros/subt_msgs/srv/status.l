;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::status)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'status (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::STATUS")
  (make-package "SUBT_MSGS::STATUS"))
(unless (find-package "SUBT_MSGS::STATUSREQUEST")
  (make-package "SUBT_MSGS::STATUSREQUEST"))
(unless (find-package "SUBT_MSGS::STATUSRESPONSE")
  (make-package "SUBT_MSGS::STATUSRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::statusRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::statusRequest
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

(defclass subt_msgs::statusResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::statusResponse
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

(defclass subt_msgs::status
  :super ros::object
  :slots ())

(setf (get subt_msgs::status :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::status :datatype-) "subt_msgs/status")
(setf (get subt_msgs::status :request) subt_msgs::statusRequest)
(setf (get subt_msgs::status :response) subt_msgs::statusResponse)

(defmethod subt_msgs::statusRequest
  (:response () (instance subt_msgs::statusResponse :init)))

(setf (get subt_msgs::statusRequest :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::statusRequest :datatype-) "subt_msgs/statusRequest")
(setf (get subt_msgs::statusRequest :definition-)
      "#Request data
---
#Response data
string result
")

(setf (get subt_msgs::statusResponse :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::statusResponse :datatype-) "subt_msgs/statusResponse")
(setf (get subt_msgs::statusResponse :definition-)
      "#Request data
---
#Response data
string result
")



(provide :subt_msgs/status "c22f2a1ed8654a0b365f1bb3f7ff2c0f")


