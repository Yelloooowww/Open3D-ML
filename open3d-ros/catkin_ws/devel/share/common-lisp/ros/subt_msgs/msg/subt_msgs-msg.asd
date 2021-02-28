
(cl:in-package :asdf)

(defsystem "subt_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AnchorBallDetection" :depends-on ("_package_AnchorBallDetection"))
    (:file "_package_AnchorBallDetection" :depends-on ("_package"))
    (:file "ArtifactPose" :depends-on ("_package_ArtifactPose"))
    (:file "_package_ArtifactPose" :depends-on ("_package"))
    (:file "ArtifactPoseArray" :depends-on ("_package_ArtifactPoseArray"))
    (:file "_package_ArtifactPoseArray" :depends-on ("_package"))
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "BoundingBoxes" :depends-on ("_package_BoundingBoxes"))
    (:file "_package_BoundingBoxes" :depends-on ("_package"))
    (:file "GloraPack" :depends-on ("_package_GloraPack"))
    (:file "_package_GloraPack" :depends-on ("_package"))
    (:file "SubTInfo" :depends-on ("_package_SubTInfo"))
    (:file "_package_SubTInfo" :depends-on ("_package"))
    (:file "arti_input" :depends-on ("_package_arti_input"))
    (:file "_package_arti_input" :depends-on ("_package"))
    (:file "bb_input" :depends-on ("_package_bb_input"))
    (:file "_package_bb_input" :depends-on ("_package"))
    (:file "mask_center" :depends-on ("_package_mask_center"))
    (:file "_package_mask_center" :depends-on ("_package"))
    (:file "masks" :depends-on ("_package_masks"))
    (:file "_package_masks" :depends-on ("_package"))
    (:file "unifiRssi" :depends-on ("_package_unifiRssi"))
    (:file "_package_unifiRssi" :depends-on ("_package"))
  ))