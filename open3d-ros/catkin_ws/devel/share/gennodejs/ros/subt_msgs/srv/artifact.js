// Auto-generated. Do not edit!

// (in-package subt_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let ArtifactPoseArray = require('../msg/ArtifactPoseArray.js');

//-----------------------------------------------------------

class artifactRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type artifactRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type artifactRequest
    let len;
    let data = new artifactRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'subt_msgs/artifactRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Request data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new artifactRequest(null);
    return resolved;
    }
};

class artifactResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.artifacts = null;
    }
    else {
      if (initObj.hasOwnProperty('artifacts')) {
        this.artifacts = initObj.artifacts
      }
      else {
        this.artifacts = new ArtifactPoseArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type artifactResponse
    // Serialize message field [artifacts]
    bufferOffset = ArtifactPoseArray.serialize(obj.artifacts, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type artifactResponse
    let len;
    let data = new artifactResponse(null);
    // Deserialize message field [artifacts]
    data.artifacts = ArtifactPoseArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ArtifactPoseArray.getMessageSize(object.artifacts);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'subt_msgs/artifactResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69bcd85d6d5bff1b077f8d902e21fd09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Response data
    ArtifactPoseArray artifacts
    
    
    ================================================================================
    MSG: subt_msgs/ArtifactPoseArray
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
    const resolved = new artifactResponse(null);
    if (msg.artifacts !== undefined) {
      resolved.artifacts = ArtifactPoseArray.Resolve(msg.artifacts)
    }
    else {
      resolved.artifacts = new ArtifactPoseArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: artifactRequest,
  Response: artifactResponse,
  md5sum() { return '69bcd85d6d5bff1b077f8d902e21fd09'; },
  datatype() { return 'subt_msgs/artifact'; }
};
