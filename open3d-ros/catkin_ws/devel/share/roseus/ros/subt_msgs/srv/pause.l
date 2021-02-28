;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::pause)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'pause (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::PAUSE")
  (make-package "SUBT_MSGS::PAUSE"))
(unless (find-package "SUBT_MSGS::PAUSEREQUEST")
  (make-package "SUBT_MSGS::PAUSEREQUEST"))
(unless (find-package "SUBT_MSGS::PAUSERESPONSE")
  (make-package "SUBT_MSGS::PAUSERESPONSE"))

(in-package "ROS")





(defclass subt_msgs::pauseRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::pauseRequest
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

(defclass subt_msgs::pauseResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::pauseResponse
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

(defclass subt_msgs::pause
  :super ros::object
  :slots ())

(setf (get subt_msgs::pause :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::pause :datatype-) "subt_msgs/pause")
(setf (get subt_msgs::pause :request) subt_msgs::pauseRequest)
(setf (get subt_msgs::pause :response) subt_msgs::pauseResponse)

(defmethod subt_msgs::pauseRequest
  (:response () (instance subt_msgs::pauseResponse :init)))

(setf (get subt_msgs::pauseRequest :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::pauseRequest :datatype-) "subt_msgs/pauseRequest")
(setf (get subt_msgs::pauseRequest :definition-)
      "#Request data
---
#Response data
string result
")

(setf (get subt_msgs::pauseResponse :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::pauseResponse :datatype-) "subt_msgs/pauseResponse")
(setf (get subt_msgs::pauseResponse :definition-)
      "#Request data
---
#Response data
string result
")



(provide :subt_msgs/pause "c22f2a1ed8654a0b365f1bb3f7ff2c0f")


