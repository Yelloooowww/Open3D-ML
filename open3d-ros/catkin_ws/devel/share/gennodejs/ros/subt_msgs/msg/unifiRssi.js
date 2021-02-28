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

class unifiRssi {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.client_name = null;
      this.connect_ap_name = null;
      this.client_rssi = null;
      this.ap_name = null;
      this.ap_rssi = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('client_name')) {
        this.client_name = initObj.client_name
      }
      else {
        this.client_name = '';
      }
      if (initObj.hasOwnProperty('connect_ap_name')) {
        this.connect_ap_name = initObj.connect_ap_name
      }
      else {
        this.connect_ap_name = '';
      }
      if (initObj.hasOwnProperty('client_rssi')) {
        this.client_rssi = initObj.client_rssi
      }
      else {
        this.client_rssi = 0;
      }
      if (initObj.hasOwnProperty('ap_name')) {
        this.ap_name = initObj.ap_name
      }
      else {
        this.ap_name = '';
      }
      if (initObj.hasOwnProperty('ap_rssi')) {
        this.ap_rssi = initObj.ap_rssi
      }
      else {
        this.ap_rssi = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type unifiRssi
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [client_name]
    bufferOffset = _serializer.string(obj.client_name, buffer, bufferOffset);
    // Serialize message field [connect_ap_name]
    bufferOffset = _serializer.string(obj.connect_ap_name, buffer, bufferOffset);
    // Serialize message field [client_rssi]
    bufferOffset = _serializer.uint16(obj.client_rssi, buffer, bufferOffset);
    // Serialize message field [ap_name]
    bufferOffset = _serializer.string(obj.ap_name, buffer, bufferOffset);
    // Serialize message field [ap_rssi]
    bufferOffset = _serializer.uint16(obj.ap_rssi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type unifiRssi
    let len;
    let data = new unifiRssi(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [client_name]
    data.client_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [connect_ap_name]
    data.connect_ap_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [client_rssi]
    data.client_rssi = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ap_name]
    data.ap_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ap_rssi]
    data.ap_rssi = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.client_name.length;
    length += object.connect_ap_name.length;
    length += object.ap_name.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'subt_msgs/unifiRssi';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9df6040c2bb377a5a4138aa41245a942';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new unifiRssi(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.client_name !== undefined) {
      resolved.client_name = msg.client_name;
    }
    else {
      resolved.client_name = ''
    }

    if (msg.connect_ap_name !== undefined) {
      resolved.connect_ap_name = msg.connect_ap_name;
    }
    else {
      resolved.connect_ap_name = ''
    }

    if (msg.client_rssi !== undefined) {
      resolved.client_rssi = msg.client_rssi;
    }
    else {
      resolved.client_rssi = 0
    }

    if (msg.ap_name !== undefined) {
      resolved.ap_name = msg.ap_name;
    }
    else {
      resolved.ap_name = ''
    }

    if (msg.ap_rssi !== undefined) {
      resolved.ap_rssi = msg.ap_rssi;
    }
    else {
      resolved.ap_rssi = 0
    }

    return resolved;
    }
};

module.exports = unifiRssi;
