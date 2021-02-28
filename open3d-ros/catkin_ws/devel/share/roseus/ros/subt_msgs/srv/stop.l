;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::stop)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'stop (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::STOP")
  (make-package "SUBT_MSGS::STOP"))
(unless (find-package "SUBT_MSGS::STOPREQUEST")
  (make-package "SUBT_MSGS::STOPREQUEST"))
(unless (find-package "SUBT_MSGS::STOPRESPONSE")
  (make-package "SUBT_MSGS::STOPRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::stopRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::stopRequest
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

(defclass subt_msgs::stopResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::stopResponse
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

(defclass subt_msgs::stop
  :super ros::object
  :slots ())

(setf (get subt_msgs::stop :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::stop :datatype-) "subt_msgs/stop")
(setf (get subt_msgs::stop :request) subt_msgs::stopRequest)
(setf (get subt_msgs::stop :response) subt_msgs::stopResponse)

(defmethod subt_msgs::stopRequest
  (:response () (instance subt_msgs::stopResponse :init)))

(setf (get subt_msgs::stopRequest :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::stopRequest :datatype-) "subt_msgs/stopRequest")
(setf (get subt_msgs::stopRequest :definition-)
      "#Request data
---
#Response data
string result
")

(setf (get subt_msgs::stopResponse :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::stopResponse :datatype-) "subt_msgs/stopResponse")
(setf (get subt_msgs::stopResponse :definition-)
      "#Request data
---
#Response data
string result
")



(provide :subt_msgs/stop "c22f2a1ed8654a0b365f1bb3f7ff2c0f")


