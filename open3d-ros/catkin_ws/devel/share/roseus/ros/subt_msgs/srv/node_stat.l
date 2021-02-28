;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::node_stat)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'node_stat (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::NODE_STAT")
  (make-package "SUBT_MSGS::NODE_STAT"))
(unless (find-package "SUBT_MSGS::NODE_STATREQUEST")
  (make-package "SUBT_MSGS::NODE_STATREQUEST"))
(unless (find-package "SUBT_MSGS::NODE_STATRESPONSE")
  (make-package "SUBT_MSGS::NODE_STATRESPONSE"))

(in-package "ROS")





(defclass subt_msgs::node_statRequest
  :super ros::object
  :slots (_count ))

(defmethod subt_msgs::node_statRequest
  (:init
   (&key
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _count (round __count))
   self)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; int32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass subt_msgs::node_statResponse
  :super ros::object
  :slots (_result ))

(defmethod subt_msgs::node_statResponse
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

(defclass subt_msgs::node_stat
  :super ros::object
  :slots ())

(setf (get subt_msgs::node_stat :md5sum-) "54387671ddc71365ab725d0391f330f3")
(setf (get subt_msgs::node_stat :datatype-) "subt_msgs/node_stat")
(setf (get subt_msgs::node_stat :request) subt_msgs::node_statRequest)
(setf (get subt_msgs::node_stat :response) subt_msgs::node_statResponse)

(defmethod subt_msgs::node_statRequest
  (:response () (instance subt_msgs::node_statResponse :init)))

(setf (get subt_msgs::node_statRequest :md5sum-) "54387671ddc71365ab725d0391f330f3")
(setf (get subt_msgs::node_statRequest :datatype-) "subt_msgs/node_statRequest")
(setf (get subt_msgs::node_statRequest :definition-)
      "#Request data
int32 count
---
#Response data
string result

")

(setf (get subt_msgs::node_statResponse :md5sum-) "54387671ddc71365ab725d0391f330f3")
(setf (get subt_msgs::node_statResponse :datatype-) "subt_msgs/node_statResponse")
(setf (get subt_msgs::node_statResponse :definition-)
      "#Request data
int32 count
---
#Response data
string result

")



(provide :subt_msgs/node_stat "54387671ddc71365ab725d0391f330f3")


