// Auto-generated. Do not edit!

// (in-package subt_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ArtifactPose = require('../msg/ArtifactPose.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class reportRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.artifact = null;
    }
    else {
      if (initObj.hasOwnProperty('artifact')) {
        this.artifact = initObj.artifact
      }
      else {
        this.artifact = new ArtifactPose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reportRequest
    // Serialize message field [artifact]
    bufferOffset = ArtifactPose.serialize(obj.artifact, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reportRequest
    let len;
    let data = new reportRequest(null);
    // Deserialize message field [artifact]
    data.artifact = ArtifactPose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ArtifactPose.getMessageSize(object.artifact);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'subt_msgs/reportRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '30502186e4f3b4efefe5fcdb32980b4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Request data
    ArtifactPose artifact
    
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
    const resolved = new reportRequest(null);
    if (msg.artifact !== undefined) {
      resolved.artifact = ArtifactPose.Resolve(msg.artifact)
    }
    else {
      resolved.artifact = new ArtifactPose()
    }

    return resolved;
    }
};

class reportResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reportResponse
    // Serialize message field [result]
    bufferOffset = _serializer.string(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reportResponse
    let len;
    let data = new reportResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.result.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'subt_msgs/reportResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c22f2a1ed8654a0b365f1bb3f7ff2c0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Response data
    string result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reportResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: reportRequest,
  Response: reportResponse,
  md5sum() { return 'd571fa3f7149c57374206fa01faa05ff'; },
  datatype() { return 'subt_msgs/report'; }
};
