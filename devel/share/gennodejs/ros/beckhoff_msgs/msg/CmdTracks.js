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

class CmdTracks {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stop_tracks = null;
      this.start_tracks = null;
      this.linear_vel = null;
      this.rot_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('stop_tracks')) {
        this.stop_tracks = initObj.stop_tracks
      }
      else {
        this.stop_tracks = false;
      }
      if (initObj.hasOwnProperty('start_tracks')) {
        this.start_tracks = initObj.start_tracks
      }
      else {
        this.start_tracks = false;
      }
      if (initObj.hasOwnProperty('linear_vel')) {
        this.linear_vel = initObj.linear_vel
      }
      else {
        this.linear_vel = 0.0;
      }
      if (initObj.hasOwnProperty('rot_vel')) {
        this.rot_vel = initObj.rot_vel
      }
      else {
        this.rot_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CmdTracks
    // Serialize message field [stop_tracks]
    bufferOffset = _serializer.bool(obj.stop_tracks, buffer, bufferOffset);
    // Serialize message field [start_tracks]
    bufferOffset = _serializer.bool(obj.start_tracks, buffer, bufferOffset);
    // Serialize message field [linear_vel]
    bufferOffset = _serializer.float64(obj.linear_vel, buffer, bufferOffset);
    // Serialize message field [rot_vel]
    bufferOffset = _serializer.float64(obj.rot_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CmdTracks
    let len;
    let data = new CmdTracks(null);
    // Deserialize message field [stop_tracks]
    data.stop_tracks = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [start_tracks]
    data.start_tracks = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [linear_vel]
    data.linear_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rot_vel]
    data.rot_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beckhoff_msgs/CmdTracks';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9944d5598f5dddfb3ecdf391f9c85ed5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool stop_tracks
    bool start_tracks
    float64 linear_vel
    float64 rot_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CmdTracks(null);
    if (msg.stop_tracks !== undefined) {
      resolved.stop_tracks = msg.stop_tracks;
    }
    else {
      resolved.stop_tracks = false
    }

    if (msg.start_tracks !== undefined) {
      resolved.start_tracks = msg.start_tracks;
    }
    else {
      resolved.start_tracks = false
    }

    if (msg.linear_vel !== undefined) {
      resolved.linear_vel = msg.linear_vel;
    }
    else {
      resolved.linear_vel = 0.0
    }

    if (msg.rot_vel !== undefined) {
      resolved.rot_vel = msg.rot_vel;
    }
    else {
      resolved.rot_vel = 0.0
    }

    return resolved;
    }
};

module.exports = CmdTracks;
