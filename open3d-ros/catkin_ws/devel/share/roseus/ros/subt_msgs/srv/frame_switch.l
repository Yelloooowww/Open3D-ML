;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::frame_switch)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'frame_switch (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::FRAME_SWITCH")
  (make-package "SUBT_MSGS::FRAME_SWITCH"))
(unless (find-package "SUBT_MSGS::FRAME_SWITCHREQUEST")
  (make-package "SUBT_MSGS::FRAME_SWITCHREQUEST"))
(unless (find-package "SUBT_MSGS::FRAME_SWITCHRESPONSE")
  (make-package "SUBT_MSGS::FRAME_SWITCHRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::frame_switchRequest
  :super ros::object
  :slots (_frame ))

(defmethod subt_msgs::frame_switchRequest
  (:init
   (&key
    ((:frame __frame) "")
    )
   (send-super :init)
   (setq _frame (string __frame))
   self)
  (:frame
   (&optional __frame)
   (if __frame (setq _frame __frame)) _frame)
  (:serialization-length
   ()
   (+
    ;; string _frame
    4 (length _frame)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame
       (write-long (length _frame) s) (princ _frame s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass subt_msgs::frame_switchResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::frame_switchResponse
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

(defclass subt_msgs::frame_switch
  :super ros::object
  :slots ())

(setf (get subt_msgs::frame_switch :md5sum-) "50cdb9fa30e06e199809b1d13df3e46b")
(setf (get subt_msgs::frame_switch :datatype-) "subt_msgs/frame_switch")
(setf (get subt_msgs::frame_switch :request) subt_msgs::frame_switchRequest)
(setf (get subt_msgs::frame_switch :response) subt_msgs::frame_switchResponse)

(defmethod subt_msgs::frame_switchRequest
  (:response () (instance subt_msgs::frame_switchResponse :init)))

(setf (get subt_msgs::frame_switchRequest :md5sum-) "50cdb9fa30e06e199809b1d13df3e46b")
(setf (get subt_msgs::frame_switchRequest :datatype-) "subt_msgs/frame_switchRequest")
(setf (get subt_msgs::frame_switchRequest :definition-)
      "#Request data
string frame # for switch frame
---
#Response data
string result # for mission result
")

(setf (get subt_msgs::frame_switchResponse :md5sum-) "50cdb9fa30e06e199809b1d13df3e46b")
(setf (get subt_msgs::frame_switchResponse :datatype-) "subt_msgs/frame_switchResponse")
(setf (get subt_msgs::frame_switchResponse :definition-)
      "#Request data
string frame # for switch frame
---
#Response data
string result # for mission result
")



(provide :subt_msgs/frame_switch "50cdb9fa30e06e199809b1d13df3e46b")


