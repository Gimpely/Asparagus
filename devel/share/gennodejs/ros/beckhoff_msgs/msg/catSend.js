// Auto-generated. Do not edit!

// (in-package beckhoff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class catSend {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.VelX = null;
      this.VelRot = null;
      this.ResetPoz = null;
    }
    else {
      if (initObj.hasOwnProperty('VelX')) {
        this.VelX = initObj.VelX
      }
      else {
        this.VelX = 0.0;
      }
      if (initObj.hasOwnProperty('VelRot')) {
        this.VelRot = initObj.VelRot
      }
      else {
        this.VelRot = 0.0;
      }
      if (initObj.hasOwnProperty('ResetPoz')) {
        this.ResetPoz = initObj.ResetPoz
      }
      else {
        this.ResetPoz = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type catSend
    // Serialize message field [VelX]
    bufferOffset = _serializer.float64(obj.VelX, buffer, bufferOffset);
    // Serialize message field [VelRot]
    bufferOffset = _serializer.float64(obj.VelRot, buffer, bufferOffset);
    // Serialize message field [ResetPoz]
    bufferOffset = _serializer.bool(obj.ResetPoz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type catSend
    let len;
    let data = new catSend(null);
    // Deserialize message field [VelX]
    data.VelX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [VelRot]
    data.VelRot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ResetPoz]
    data.ResetPoz = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beckhoff_msgs/catSend';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e4b9f26ff50bf503fcd6a14b54ee63d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 VelX
    float64 VelRot
    bool ResetPoz
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new catSend(null);
    if (msg.VelX !== undefined) {
      resolved.VelX = msg.VelX;
    }
    else {
      resolved.VelX = 0.0
    }

    if (msg.VelRot !== undefined) {
      resolved.VelRot = msg.VelRot;
    }
    else {
      resolved.VelRot = 0.0
    }

    if (msg.ResetPoz !== undefined) {
      resolved.ResetPoz = msg.ResetPoz;
    }
    else {
      resolved.ResetPoz = false
    }

    return resolved;
    }
};

module.exports = catSend;
