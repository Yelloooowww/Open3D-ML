;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::unifiRssi)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'unifiRssi (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::UNIFIRSSI")
  (make-package "SUBT_MSGS::UNIFIRSSI"))

(in-package "ROS")
;;//! \htmlinclude unifiRssi.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass subt_msgs::unifiRssi
  :super ros::object
  :slots (_header _client_name _connect_ap_name _client_rssi _ap_name _ap_rssi ))

(defmethod subt_msgs::unifiRssi
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:client_name __client_name) "")
    ((:connect_ap_name __connect_ap_name) "")
    ((:client_rssi __client_rssi) 0)
    ((:ap_name __ap_name) "")
    ((:ap_rssi __ap_rssi) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _client_name (string __client_name))
   (setq _connect_ap_name (string __connect_ap_name))
   (setq _client_rssi (round __client_rssi))
   (setq _ap_name (string __ap_name))
   (setq _ap_rssi (round __ap_rssi))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:client_name
   (&optional __client_name)
   (if __client_name (setq _client_name __client_name)) _client_name)
  (:connect_ap_name
   (&optional __connect_ap_name)
   (if __connect_ap_name (setq _connect_ap_name __connect_ap_name)) _connect_ap_name)
  (:client_rssi
   (&optional __client_rssi)
   (if __client_rssi (setq _client_rssi __client_rssi)) _client_rssi)
  (:ap_name
   (&optional __ap_name)
   (if __ap_name (setq _ap_name __ap_name)) _ap_name)
  (:ap_rssi
   (&optional __ap_rssi)
   (if __ap_rssi (setq _ap_rssi __ap_rssi)) _ap_rssi)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _client_name
    4 (length _client_name)
    ;; string _connect_ap_name
    4 (length _connect_ap_name)
    ;; uint16 _client_rssi
    2
    ;; string _ap_name
    4 (length _ap_name)
    ;; uint16 _ap_rssi
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _client_name
       (write-long (length _client_name) s) (princ _client_name s)
     ;; string _connect_ap_name
       (write-long (length _connect_ap_name) s) (princ _connect_ap_name s)
     ;; uint16 _client_rssi
       (write-word _client_rssi s)
     ;; string _ap_name
       (write-long (length _ap_name) s) (princ _ap_name s)
     ;; uint16 _ap_rssi
       (write-word _ap_rssi s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _client_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _client_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _connect_ap_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _connect_ap_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint16 _client_rssi
     (setq _client_rssi (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _ap_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ap_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint16 _ap_rssi
     (setq _ap_rssi (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get subt_msgs::unifiRssi :md5sum-) "9df6040c2bb377a5a4138aa41245a942")
(setf (get subt_msgs::unifiRssi :datatype-) "subt_msgs/unifiRssi")
(setf (get subt_msgs::unifiRssi :definition-)
      "Header header

string client_name
string connect_ap_name
uint16 client_rssi
string ap_name
uint16 ap_rssi

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



(provide :subt_msgs/unifiRssi "9df6040c2bb377a5a4138aa41245a942")


