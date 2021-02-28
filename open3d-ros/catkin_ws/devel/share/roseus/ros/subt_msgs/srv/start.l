;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::start)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'start (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::START")
  (make-package "SUBT_MSGS::START"))
(unless (find-package "SUBT_MSGS::STARTREQUEST")
  (make-package "SUBT_MSGS::STARTREQUEST"))
(unless (find-package "SUBT_MSGS::STARTRESPONSE")
  (make-package "SUBT_MSGS::STARTRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::startRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::startRequest
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

(defclass subt_msgs::startResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::startResponse
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

(defclass subt_msgs::start
  :super ros::object
  :slots ())

(setf (get subt_msgs::start :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::start :datatype-) "subt_msgs/start")
(setf (get subt_msgs::start :request) subt_msgs::startRequest)
(setf (get subt_msgs::start :response) subt_msgs::startResponse)

(defmethod subt_msgs::startRequest
  (:response () (instance subt_msgs::startResponse :init)))

(setf (get subt_msgs::startRequest :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::startRequest :datatype-) "subt_msgs/startRequest")
(setf (get subt_msgs::startRequest :definition-)
      "#Request data
---
#Response data
string result
")

(setf (get subt_msgs::startResponse :md5sum-) "c22f2a1ed8654a0b365f1bb3f7ff2c0f")
(setf (get subt_msgs::startResponse :datatype-) "subt_msgs/startResponse")
(setf (get subt_msgs::startResponse :definition-)
      "#Request data
---
#Response data
string result
")



(provide :subt_msgs/start "c22f2a1ed8654a0b365f1bb3f7ff2c0f")


