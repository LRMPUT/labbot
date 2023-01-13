// Auto-generated. Do not edit!

// (in-package labbot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class msgToLabbot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motorRightSpeed = null;
      this.motorLeftSpeed = null;
    }
    else {
      if (initObj.hasOwnProperty('motorRightSpeed')) {
        this.motorRightSpeed = initObj.motorRightSpeed
      }
      else {
        this.motorRightSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('motorLeftSpeed')) {
        this.motorLeftSpeed = initObj.motorLeftSpeed
      }
      else {
        this.motorLeftSpeed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgToLabbot
    // Serialize message field [motorRightSpeed]
    bufferOffset = _serializer.float32(obj.motorRightSpeed, buffer, bufferOffset);
    // Serialize message field [motorLeftSpeed]
    bufferOffset = _serializer.float32(obj.motorLeftSpeed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgToLabbot
    let len;
    let data = new msgToLabbot(null);
    // Deserialize message field [motorRightSpeed]
    data.motorRightSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorLeftSpeed]
    data.motorLeftSpeed = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'labbot/msgToLabbot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '227e24617faca74e2a267f0d326d13ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #motor speed (range: 0-80)
    float32 motorRightSpeed
    float32 motorLeftSpeed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgToLabbot(null);
    if (msg.motorRightSpeed !== undefined) {
      resolved.motorRightSpeed = msg.motorRightSpeed;
    }
    else {
      resolved.motorRightSpeed = 0.0
    }

    if (msg.motorLeftSpeed !== undefined) {
      resolved.motorLeftSpeed = msg.motorLeftSpeed;
    }
    else {
      resolved.motorLeftSpeed = 0.0
    }

    return resolved;
    }
};

module.exports = msgToLabbot;
