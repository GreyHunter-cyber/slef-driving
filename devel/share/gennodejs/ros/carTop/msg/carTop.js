// Auto-generated. Do not edit!

// (in-package carTop.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class carTop {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.back_wheel_speed = null;
      this.turn_angle = null;
      this.battery_level = null;
      this.error_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('back_wheel_speed')) {
        this.back_wheel_speed = initObj.back_wheel_speed
      }
      else {
        this.back_wheel_speed = 0.0;
      }
      if (initObj.hasOwnProperty('turn_angle')) {
        this.turn_angle = initObj.turn_angle
      }
      else {
        this.turn_angle = 0.0;
      }
      if (initObj.hasOwnProperty('battery_level')) {
        this.battery_level = initObj.battery_level
      }
      else {
        this.battery_level = 0;
      }
      if (initObj.hasOwnProperty('error_flag')) {
        this.error_flag = initObj.error_flag
      }
      else {
        this.error_flag = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type carTop
    // Serialize message field [back_wheel_speed]
    bufferOffset = _serializer.float64(obj.back_wheel_speed, buffer, bufferOffset);
    // Serialize message field [turn_angle]
    bufferOffset = _serializer.float64(obj.turn_angle, buffer, bufferOffset);
    // Serialize message field [battery_level]
    bufferOffset = _serializer.char(obj.battery_level, buffer, bufferOffset);
    // Serialize message field [error_flag]
    bufferOffset = _serializer.char(obj.error_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type carTop
    let len;
    let data = new carTop(null);
    // Deserialize message field [back_wheel_speed]
    data.back_wheel_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [turn_angle]
    data.turn_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [battery_level]
    data.battery_level = _deserializer.char(buffer, bufferOffset);
    // Deserialize message field [error_flag]
    data.error_flag = _deserializer.char(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'carTop/carTop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2399edfa2a8160084c724032695e45ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 back_wheel_speed
    float64 turn_angle
    char battery_level
    char error_flag
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new carTop(null);
    if (msg.back_wheel_speed !== undefined) {
      resolved.back_wheel_speed = msg.back_wheel_speed;
    }
    else {
      resolved.back_wheel_speed = 0.0
    }

    if (msg.turn_angle !== undefined) {
      resolved.turn_angle = msg.turn_angle;
    }
    else {
      resolved.turn_angle = 0.0
    }

    if (msg.battery_level !== undefined) {
      resolved.battery_level = msg.battery_level;
    }
    else {
      resolved.battery_level = 0
    }

    if (msg.error_flag !== undefined) {
      resolved.error_flag = msg.error_flag;
    }
    else {
      resolved.error_flag = 0
    }

    return resolved;
    }
};

module.exports = carTop;
