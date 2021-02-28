// Auto-generated. Do not edit!

// (in-package subt_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GloraPack {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.packet = null;
      this.time_elapse = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('packet')) {
        this.packet = initObj.packet
      }
      else {
        this.packet = '';
      }
      if (initObj.hasOwnProperty('time_elapse')) {
        this.time_elapse = initObj.time_elapse
      }
      else {
        this.time_elapse = 0.0;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GloraPack
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [packet]
    bufferOffset = _serializer.string(obj.packet, buffer, bufferOffset);
    // Serialize message field [time_elapse]
    bufferOffset = _serializer.float32(obj.time_elapse, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GloraPack
    let len;
    let data = new GloraPack(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [packet]
    data.packet = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [time_elapse]
    data.time_elapse = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.packet.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'subt_msgs/GloraPack';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '793b4359128b7f4b7385d49cb96eaaec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GloraPack(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.packet !== undefined) {
      resolved.packet = msg.packet;
    }
    else {
      resolved.packet = ''
    }

    if (msg.time_elapse !== undefined) {
      resolved.time_elapse = msg.time_elapse;
    }
    else {
      resolved.time_elapse = 0.0
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = GloraPack;
