// Auto-generated. Do not edit!

// (in-package lslidar_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class lslidar_controlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.LaserControl = null;
    }
    else {
      if (initObj.hasOwnProperty('LaserControl')) {
        this.LaserControl = initObj.LaserControl
      }
      else {
        this.LaserControl = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lslidar_controlRequest
    // Serialize message field [LaserControl]
    bufferOffset = _serializer.int32(obj.LaserControl, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lslidar_controlRequest
    let len;
    let data = new lslidar_controlRequest(null);
    // Deserialize message field [LaserControl]
    data.LaserControl = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lslidar_msgs/lslidar_controlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '092ff9dfd0fc90557976e746314fce5a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 LaserControl
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lslidar_controlRequest(null);
    if (msg.LaserControl !== undefined) {
      resolved.LaserControl = msg.LaserControl;
    }
    else {
      resolved.LaserControl = 0
    }

    return resolved;
    }
};

class lslidar_controlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lslidar_controlResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lslidar_controlResponse
    let len;
    let data = new lslidar_controlResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lslidar_msgs/lslidar_controlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1255d4d998bd4d6585c64639b5ee9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lslidar_controlResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: lslidar_controlRequest,
  Response: lslidar_controlResponse,
  md5sum() { return '0d2a2bc3bb3a258db25b2d8c05ffa5d1'; },
  datatype() { return 'lslidar_msgs/lslidar_control'; }
};
