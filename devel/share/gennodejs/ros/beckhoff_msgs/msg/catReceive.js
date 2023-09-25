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

class catReceive {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.v = null;
      this.Velocity = null;
      this.AngularVel = null;
      this.X_poz = null;
      this.Y_poz = null;
      this.Th_poz = null;
      this.Th_pozPi = null;
    }
    else {
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('Velocity')) {
        this.Velocity = initObj.Velocity
      }
      else {
        this.Velocity = 0.0;
      }
      if (initObj.hasOwnProperty('AngularVel')) {
        this.AngularVel = initObj.AngularVel
      }
      else {
        this.AngularVel = 0.0;
      }
      if (initObj.hasOwnProperty('X_poz')) {
        this.X_poz = initObj.X_poz
      }
      else {
        this.X_poz = 0.0;
      }
      if (initObj.hasOwnProperty('Y_poz')) {
        this.Y_poz = initObj.Y_poz
      }
      else {
        this.Y_poz = 0.0;
      }
      if (initObj.hasOwnProperty('Th_poz')) {
        this.Th_poz = initObj.Th_poz
      }
      else {
        this.Th_poz = 0.0;
      }
      if (initObj.hasOwnProperty('Th_pozPi')) {
        this.Th_pozPi = initObj.Th_pozPi
      }
      else {
        this.Th_pozPi = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type catReceive
    // Check that the constant length array field [v] has the right length
    if (obj.v.length !== 2) {
      throw new Error('Unable to serialize array field v - length must be 2')
    }
    // Serialize message field [v]
    bufferOffset = _arraySerializer.float64(obj.v, buffer, bufferOffset, 2);
    // Serialize message field [Velocity]
    bufferOffset = _serializer.float64(obj.Velocity, buffer, bufferOffset);
    // Serialize message field [AngularVel]
    bufferOffset = _serializer.float64(obj.AngularVel, buffer, bufferOffset);
    // Serialize message field [X_poz]
    bufferOffset = _serializer.float64(obj.X_poz, buffer, bufferOffset);
    // Serialize message field [Y_poz]
    bufferOffset = _serializer.float64(obj.Y_poz, buffer, bufferOffset);
    // Serialize message field [Th_poz]
    bufferOffset = _serializer.float64(obj.Th_poz, buffer, bufferOffset);
    // Serialize message field [Th_pozPi]
    bufferOffset = _serializer.float64(obj.Th_pozPi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type catReceive
    let len;
    let data = new catReceive(null);
    // Deserialize message field [v]
    data.v = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [Velocity]
    data.Velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [AngularVel]
    data.AngularVel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [X_poz]
    data.X_poz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Y_poz]
    data.Y_poz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Th_poz]
    data.Th_poz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Th_pozPi]
    data.Th_pozPi = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beckhoff_msgs/catReceive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e32c3fba4c7a01b6dd0b032a6c34aeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[2] v
    float64 Velocity
    float64 AngularVel
    float64 X_poz
    float64 Y_poz
    float64 Th_poz
    float64 Th_pozPi
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new catReceive(null);
    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = new Array(2).fill(0)
    }

    if (msg.Velocity !== undefined) {
      resolved.Velocity = msg.Velocity;
    }
    else {
      resolved.Velocity = 0.0
    }

    if (msg.AngularVel !== undefined) {
      resolved.AngularVel = msg.AngularVel;
    }
    else {
      resolved.AngularVel = 0.0
    }

    if (msg.X_poz !== undefined) {
      resolved.X_poz = msg.X_poz;
    }
    else {
      resolved.X_poz = 0.0
    }

    if (msg.Y_poz !== undefined) {
      resolved.Y_poz = msg.Y_poz;
    }
    else {
      resolved.Y_poz = 0.0
    }

    if (msg.Th_poz !== undefined) {
      resolved.Th_poz = msg.Th_poz;
    }
    else {
      resolved.Th_poz = 0.0
    }

    if (msg.Th_pozPi !== undefined) {
      resolved.Th_pozPi = msg.Th_pozPi;
    }
    else {
      resolved.Th_pozPi = 0.0
    }

    return resolved;
    }
};

module.exports = catReceive;
