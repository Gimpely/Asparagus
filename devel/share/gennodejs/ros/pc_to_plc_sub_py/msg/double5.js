// Auto-generated. Do not edit!

// (in-package pc_to_plc_sub_py.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class double5 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.alpha = null;
    }
    else {
      if (initObj.hasOwnProperty('alpha')) {
        this.alpha = initObj.alpha
      }
      else {
        this.alpha = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type double5
    // Check that the constant length array field [alpha] has the right length
    if (obj.alpha.length !== 5) {
      throw new Error('Unable to serialize array field alpha - length must be 5')
    }
    // Serialize message field [alpha]
    bufferOffset = _arraySerializer.float64(obj.alpha, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type double5
    let len;
    let data = new double5(null);
    // Deserialize message field [alpha]
    data.alpha = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pc_to_plc_sub_py/double5';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc8dffcd3a49516a92785ed44605ab1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[5] alpha
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new double5(null);
    if (msg.alpha !== undefined) {
      resolved.alpha = msg.alpha;
    }
    else {
      resolved.alpha = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = double5;
