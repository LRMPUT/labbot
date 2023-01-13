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

class msgFromLabbot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motorRightInput = null;
      this.motorRightSetpoint = null;
      this.motorRightOutput = null;
      this.motorLeftInput = null;
      this.motorLeftSetpoint = null;
      this.motorLefttOutput = null;
    }
    else {
      if (initObj.hasOwnProperty('motorRightInput')) {
        this.motorRightInput = initObj.motorRightInput
      }
      else {
        this.motorRightInput = 0.0;
      }
      if (initObj.hasOwnProperty('motorRightSetpoint')) {
        this.motorRightSetpoint = initObj.motorRightSetpoint
      }
      else {
        this.motorRightSetpoint = 0.0;
      }
      if (initObj.hasOwnProperty('motorRightOutput')) {
        this.motorRightOutput = initObj.motorRightOutput
      }
      else {
        this.motorRightOutput = 0.0;
      }
      if (initObj.hasOwnProperty('motorLeftInput')) {
        this.motorLeftInput = initObj.motorLeftInput
      }
      else {
        this.motorLeftInput = 0.0;
      }
      if (initObj.hasOwnProperty('motorLeftSetpoint')) {
        this.motorLeftSetpoint = initObj.motorLeftSetpoint
      }
      else {
        this.motorLeftSetpoint = 0.0;
      }
      if (initObj.hasOwnProperty('motorLefttOutput')) {
        this.motorLefttOutput = initObj.motorLefttOutput
      }
      else {
        this.motorLefttOutput = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgFromLabbot
    // Serialize message field [motorRightInput]
    bufferOffset = _serializer.float32(obj.motorRightInput, buffer, bufferOffset);
    // Serialize message field [motorRightSetpoint]
    bufferOffset = _serializer.float32(obj.motorRightSetpoint, buffer, bufferOffset);
    // Serialize message field [motorRightOutput]
    bufferOffset = _serializer.float32(obj.motorRightOutput, buffer, bufferOffset);
    // Serialize message field [motorLeftInput]
    bufferOffset = _serializer.float32(obj.motorLeftInput, buffer, bufferOffset);
    // Serialize message field [motorLeftSetpoint]
    bufferOffset = _serializer.float32(obj.motorLeftSetpoint, buffer, bufferOffset);
    // Serialize message field [motorLefttOutput]
    bufferOffset = _serializer.float32(obj.motorLefttOutput, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgFromLabbot
    let len;
    let data = new msgFromLabbot(null);
    // Deserialize message field [motorRightInput]
    data.motorRightInput = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorRightSetpoint]
    data.motorRightSetpoint = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorRightOutput]
    data.motorRightOutput = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorLeftInput]
    data.motorLeftInput = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorLeftSetpoint]
    data.motorLeftSetpoint = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorLefttOutput]
    data.motorLefttOutput = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'labbot/msgFromLabbot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4bf0f6e2fd2b507f5c1d51ceec5b93b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # right motor parameters
    float32 motorRightInput
    float32 motorRightSetpoint
    float32 motorRightOutput
    # left motor parameters
    float32 motorLeftInput
    float32 motorLeftSetpoint
    float32 motorLefttOutput
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgFromLabbot(null);
    if (msg.motorRightInput !== undefined) {
      resolved.motorRightInput = msg.motorRightInput;
    }
    else {
      resolved.motorRightInput = 0.0
    }

    if (msg.motorRightSetpoint !== undefined) {
      resolved.motorRightSetpoint = msg.motorRightSetpoint;
    }
    else {
      resolved.motorRightSetpoint = 0.0
    }

    if (msg.motorRightOutput !== undefined) {
      resolved.motorRightOutput = msg.motorRightOutput;
    }
    else {
      resolved.motorRightOutput = 0.0
    }

    if (msg.motorLeftInput !== undefined) {
      resolved.motorLeftInput = msg.motorLeftInput;
    }
    else {
      resolved.motorLeftInput = 0.0
    }

    if (msg.motorLeftSetpoint !== undefined) {
      resolved.motorLeftSetpoint = msg.motorLeftSetpoint;
    }
    else {
      resolved.motorLeftSetpoint = 0.0
    }

    if (msg.motorLefttOutput !== undefined) {
      resolved.motorLefttOutput = msg.motorLefttOutput;
    }
    else {
      resolved.motorLefttOutput = 0.0
    }

    return resolved;
    }
};

module.exports = msgFromLabbot;
