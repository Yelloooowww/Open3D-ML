// Auto-generated. Do not edit!

// (in-package subt_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ArtifactPoseArray = require('./ArtifactPoseArray.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SubTInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robot_name = null;
      this.robot_pose = null;
      this.artifacts = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = '';
      }
      if (initObj.hasOwnProperty('robot_pose')) {
        this.robot_pose = initObj.robot_pose
      }
      else {
        this.robot_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('artifacts')) {
        this.artifacts = initObj.artifacts
      }
      else {
        this.artifacts = new ArtifactPoseArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SubTInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robot_name]
    bufferOffset = _serializer.string(obj.robot_name, buffer, bufferOffset);
    // Serialize message field [robot_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.robot_pose, buffer, bufferOffset);
    // Serialize message field [artifacts]
    bufferOffset = ArtifactPoseArray.serialize(obj.artifacts, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SubTInfo
    let len;
    let data = new SubTInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_name]
    data.robot_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [robot_pose]
    data.robot_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [artifacts]
    data.artifacts = ArtifactPoseArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.robot_name.length;
    length += ArtifactPoseArray.getMessageSize(object.artifacts);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'subt_msgs/SubTInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e821f52944c112ea518ab90db2ed050a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string robot_name
    geometry_msgs/Pose robot_pose
    ArtifactPoseArray artifacts
    #AnchorBallDetection anchorballs
    #bool has_ball
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: subt_msgs/ArtifactPoseArray
    Header header
    string camera
    int32 count
    ArtifactPose[] pose_array
    ================================================================================
    MSG: subt_msgs/ArtifactPose
    string Class
    #string status				# O for non_return , X for return 
    int32 appear_count
    float64 probability
    geometry_msgs/Pose pose
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SubTInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.robot_name !== undefined) {
      resolved.robot_name = msg.robot_name;
    }
    else {
      resolved.robot_name = ''
    }

    if (msg.robot_pose !== undefined) {
      resolved.robot_pose = geometry_msgs.msg.Pose.Resolve(msg.robot_pose)
    }
    else {
      resolved.robot_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.artifacts !== undefined) {
      resolved.artifacts = ArtifactPoseArray.Resolve(msg.artifacts)
    }
    else {
      resolved.artifacts = new ArtifactPoseArray()
    }

    return resolved;
    }
};

module.exports = SubTInfo;
