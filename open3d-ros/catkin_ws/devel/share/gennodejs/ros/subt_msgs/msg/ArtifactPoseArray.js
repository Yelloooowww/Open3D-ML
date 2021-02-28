// Auto-generated. Do not edit!

// (in-package subt_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ArtifactPose = require('./ArtifactPose.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArtifactPoseArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.camera = null;
      this.count = null;
      this.pose_array = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('camera')) {
        this.camera = initObj.camera
      }
      else {
        this.camera = '';
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
      if (initObj.hasOwnProperty('pose_array')) {
        this.pose_array = initObj.pose_array
      }
      else {
        this.pose_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArtifactPoseArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [camera]
    bufferOffset = _serializer.string(obj.camera, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.int32(obj.count, buffer, bufferOffset);
    // Serialize message field [pose_array]
    // Serialize the length for message field [pose_array]
    bufferOffset = _serializer.uint32(obj.pose_array.length, buffer, bufferOffset);
    obj.pose_array.forEach((val) => {
      bufferOffset = ArtifactPose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArtifactPoseArray
    let len;
    let data = new ArtifactPoseArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [camera]
    data.camera = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pose_array]
    // Deserialize array length for message field [pose_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pose_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pose_array[i] = ArtifactPose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.camera.length;
    object.pose_array.forEach((val) => {
      length += ArtifactPose.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'subt_msgs/ArtifactPoseArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1241fdc583a1d4a260d1fbeb9ee080f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string camera
    int32 count
    ArtifactPose[] pose_array
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
    MSG: subt_msgs/ArtifactPose
    string Class
    #string status				# O for non_return , X for return 
    int32 appear_count
    float64 probability
    geometry_msgs/Pose pose
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArtifactPoseArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.camera !== undefined) {
      resolved.camera = msg.camera;
    }
    else {
      resolved.camera = ''
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    if (msg.pose_array !== undefined) {
      resolved.pose_array = new Array(msg.pose_array.length);
      for (let i = 0; i < resolved.pose_array.length; ++i) {
        resolved.pose_array[i] = ArtifactPose.Resolve(msg.pose_array[i]);
      }
    }
    else {
      resolved.pose_array = []
    }

    return resolved;
    }
};

module.exports = ArtifactPoseArray;
