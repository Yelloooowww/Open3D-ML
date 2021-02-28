;; Auto-generated. Do not edit!


(when (boundp 'subt_msgs::masks)
  (if (not (find-package "SUBT_MSGS"))
    (make-package "SUBT_MSGS"))
  (shadow 'masks (find-package "SUBT_MSGS")))
(unless (find-package "SUBT_MSGS::MASKS")
  (make-package "SUBT_MSGS::MASKS"))

(in-package "ROS")
;;//! \htmlinclude masks.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass subt_msgs::masks
  :super ros::object
  :slots (_header _camera _count _masks _depth ))

(defmethod subt_msgs::masks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera __camera) "")
    ((:count __count) 0)
    ((:masks __masks) (let (r) (dotimes (i 0) (push (instance subt_msgs::mask_center :init) r)) r))
    ((:depth __depth) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera (string __camera))
   (setq _count (round __count))
   (setq _masks __masks)
   (setq _depth __depth)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:camera
   (&optional __camera)
   (if __camera (setq _camera __camera)) _camera)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:masks
   (&rest __masks)
   (if (keywordp (car __masks))
       (send* _masks __masks)
     (progn
       (if __masks (setq _masks (car __masks)))
       _masks)))
  (:depth
   (&rest __depth)
   (if (keywordp (car __depth))
       (send* _depth __depth)
     (progn
       (if __depth (setq _depth (car __depth)))
       _depth)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _camera
    4 (length _camera)
    ;; int32 _count
    4
    ;; subt_msgs/mask_center[] _masks
    (apply #'+ (send-all _masks :serialization-length)) 4
    ;; sensor_msgs/Image _depth
    (send _depth :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _camera
       (write-long (length _camera) s) (princ _camera s)
     ;; int32 _count
       (write-long _count s)
     ;; subt_msgs/mask_center[] _masks
     (write-long (length _masks) s)
     (dolist (elem _masks)
       (send elem :serialize s)
       )
     ;; sensor_msgs/Image _depth
       (send _depth :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _camera
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; subt_msgs/mask_center[] _masks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _masks (let (r) (dotimes (i n) (push (instance subt_msgs::mask_center :init) r)) r))
     (dolist (elem- _masks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; sensor_msgs/Image _depth
     (send _depth :deserialize buf ptr-) (incf ptr- (send _depth :serialization-length))
   ;;
   self)
  )

(setf (get subt_msgs::masks :md5sum-) "30b0cc6ee1e247590a77fe90859cf2c4")
(setf (get subt_msgs::masks :datatype-) "subt_msgs/masks")
(setf (get subt_msgs::masks :definition-)
      "Header header
string camera
int32 count
mask_center[] masks
sensor_msgs/Image depth
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

================================================================================
MSG: subt_msgs/mask_center
string Class
int64 x
int64 y
int64 z
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

")



(provide :subt_msgs/masks "30b0cc6ee1e247590a77fe90859cf2c4")


