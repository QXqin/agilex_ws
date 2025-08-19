// Auto-generated. Do not edit!

// (in-package lifter_ctr.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class lifter_mode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.updown_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('updown_mode')) {
        this.updown_mode = initObj.updown_mode
      }
      else {
        this.updown_mode = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lifter_mode
    // Serialize message field [updown_mode]
    bufferOffset = _serializer.string(obj.updown_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lifter_mode
    let len;
    let data = new lifter_mode(null);
    // Deserialize message field [updown_mode]
    data.updown_mode = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.updown_mode.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lifter_ctr/lifter_mode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7990a6d0129a755d88d46f3f5051aed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string updown_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lifter_mode(null);
    if (msg.updown_mode !== undefined) {
      resolved.updown_mode = msg.updown_mode;
    }
    else {
      resolved.updown_mode = ''
    }

    return resolved;
    }
};

module.exports = lifter_mode;
