;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::GloraPack)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'GloraPack (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::GLORAPACK")
  (make-package "SUBT_MSGS::GLORAPACK"))

(in-package "ROS")
;;//! \htmlinclude GloraPack.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass subt_msgs::GloraPack
  :super ros::object
  :slots (_header _packet _time_elapse _success ))

(defmethod subt_msgs::GloraPack
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:packet __packet) "")
    ((:time_elapse __time_elapse) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _packet (string __packet))
   (setq _time_elapse (float __time_elapse))
   (setq _success __success)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:packet
   (&optional __packet)
   (if __packet (setq _packet __packet)) _packet)
  (:time_elapse
   (&optional __time_elapse)
   (if __time_elapse (setq _time_elapse __time_elapse)) _time_elapse)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _packet
    4 (length _packet)
    ;; float32 _time_elapse
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _packet
       (write-long (length _packet) s) (princ _packet s)
     ;; float32 _time_elapse
       (sys::poke _time_elapse (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _packet
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _packet (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _time_elapse
     (setq _time_elapse (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get subt_msgs::GloraPack :md5sum-) "793b4359128b7f4b7385d49cb96eaaec")
(setf (get subt_msgs::GloraPack :datatype-) "subt_msgs/GloraPack")
(setf (get subt_msgs::GloraPack :definition-)
      "Header header
string packet
float32 time_elapse 
bool success

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :subt_msgs/GloraPack "793b4359128b7f4b7385d49cb96eaaec")


