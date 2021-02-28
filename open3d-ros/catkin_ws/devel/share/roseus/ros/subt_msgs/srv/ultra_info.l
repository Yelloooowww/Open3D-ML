;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::ultra_info)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'ultra_info (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::ULTRA_INFO")
  (make-package "SUBT_MSGS::ULTRA_INFO"))
(unless (find-package "SUBT_MSGS::ULTRA_INFOREQUEST")
  (make-package "SUBT_MSGS::ULTRA_INFOREQUEST"))
(unless (find-package "SUBT_MSGS::ULTRA_INFORESPONSE")
  (make-package "SUBT_MSGS::ULTRA_INFORESPONSE"))

(in-package "ROS")





(defclass subt_msgs::ultra_infoRequest
  :super ros::object
  :slots ())

(defmethod subt_msgs::ultra_infoRequest
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

(defclass subt_msgs::ultra_infoResponse
  :super ros::object
  :slots (_status _height ))

(defmethod subt_msgs::ultra_infoResponse
  (:init
   (&key
    ((:status __status) nil)
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _status __status)
   (setq _height (float __height))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ;; float32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass subt_msgs::ultra_info
  :super ros::object
  :slots ())

(setf (get subt_msgs::ultra_info :md5sum-) "839104ea8fff48a8d67dee52c3b6127a")
(setf (get subt_msgs::ultra_info :datatype-) "subt_msgs/ultra_info")
(setf (get subt_msgs::ultra_info :request) subt_msgs::ultra_infoRequest)
(setf (get subt_msgs::ultra_info :response) subt_msgs::ultra_infoResponse)

(defmethod subt_msgs::ultra_infoRequest
  (:response () (instance subt_msgs::ultra_infoResponse :init)))

(setf (get subt_msgs::ultra_infoRequest :md5sum-) "839104ea8fff48a8d67dee52c3b6127a")
(setf (get subt_msgs::ultra_infoRequest :datatype-) "subt_msgs/ultra_infoRequest")
(setf (get subt_msgs::ultra_infoRequest :definition-)
      "#Request data
---
#Response data
bool status		# True for normal, False for danger
float32 height
# string result # for mission result
")

(setf (get subt_msgs::ultra_infoResponse :md5sum-) "839104ea8fff48a8d67dee52c3b6127a")
(setf (get subt_msgs::ultra_infoResponse :datatype-) "subt_msgs/ultra_infoResponse")
(setf (get subt_msgs::ultra_infoResponse :definition-)
      "#Request data
---
#Response data
bool status		# True for normal, False for danger
float32 height
# string result # for mission result
")



(provide :subt_msgs/ultra_info "839104ea8fff48a8d67dee52c3b6127a")


