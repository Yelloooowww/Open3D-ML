;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::int8)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'int8 (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::INT8")
  (make-package "SUBT_MSGS::INT8"))
(unless (find-package "SUBT_MSGS::INT8REQUEST")
  (make-package "SUBT_MSGS::INT8REQUEST"))
(unless (find-package "SUBT_MSGS::INT8RESPONSE")
  (make-package "SUBT_MSGS::INT8RESPONSE"))

(in-package "ROS")





(defclass subt_msgs::int8Request
  :super ros::object
  :slots (_data ))

(defmethod subt_msgs::int8Request
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _data 127) (setq _data (- _data 256)))
   ;;
   self)
  )

(defclass subt_msgs::int8Response
  :super ros::object
  :slots ())

(defmethod subt_msgs::int8Response
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

(defclass subt_msgs::int8
  :super ros::object
  :slots ())

(setf (get subt_msgs::int8 :md5sum-) "27ffa0c9c4b8fb8492252bcad9e5c57b")
(setf (get subt_msgs::int8 :datatype-) "subt_msgs/int8")
(setf (get subt_msgs::int8 :request) subt_msgs::int8Request)
(setf (get subt_msgs::int8 :response) subt_msgs::int8Response)

(defmethod subt_msgs::int8Request
  (:response () (instance subt_msgs::int8Response :init)))

(setf (get subt_msgs::int8Request :md5sum-) "27ffa0c9c4b8fb8492252bcad9e5c57b")
(setf (get subt_msgs::int8Request :datatype-) "subt_msgs/int8Request")
(setf (get subt_msgs::int8Request :definition-)
      "#Request data
int8 data
---
#Response data
# string result # for mission result
")

(setf (get subt_msgs::int8Response :md5sum-) "27ffa0c9c4b8fb8492252bcad9e5c57b")
(setf (get subt_msgs::int8Response :datatype-) "subt_msgs/int8Response")
(setf (get subt_msgs::int8Response :definition-)
      "#Request data
int8 data
---
#Response data
# string result # for mission result
")



(provide :subt_msgs/int8 "27ffa0c9c4b8fb8492252bcad9e5c57b")


