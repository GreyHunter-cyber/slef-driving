// Auto-generated. Do not edit!

// (in-package plan_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PointTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.t = null;
      this.d = null;
      this.d_d = null;
      this.d_dd = null;
      this.d_ddd = null;
      this.s = null;
      this.s_d = null;
      this.s_dd = null;
      this.s_ddd = null;
      this.x = null;
      this.y = null;
      this.yaw = null;
      this.ks = null;
      this.lx = null;
      this.ly = null;
      this.lyaw = null;
    }
    else {
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = 0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('d_d')) {
        this.d_d = initObj.d_d
      }
      else {
        this.d_d = 0.0;
      }
      if (initObj.hasOwnProperty('d_dd')) {
        this.d_dd = initObj.d_dd
      }
      else {
        this.d_dd = 0.0;
      }
      if (initObj.hasOwnProperty('d_ddd')) {
        this.d_ddd = initObj.d_ddd
      }
      else {
        this.d_ddd = 0.0;
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = 0.0;
      }
      if (initObj.hasOwnProperty('s_d')) {
        this.s_d = initObj.s_d
      }
      else {
        this.s_d = 0.0;
      }
      if (initObj.hasOwnProperty('s_dd')) {
        this.s_dd = initObj.s_dd
      }
      else {
        this.s_dd = 0.0;
      }
      if (initObj.hasOwnProperty('s_ddd')) {
        this.s_ddd = initObj.s_ddd
      }
      else {
        this.s_ddd = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('ks')) {
        this.ks = initObj.ks
      }
      else {
        this.ks = 0.0;
      }
      if (initObj.hasOwnProperty('lx')) {
        this.lx = initObj.lx
      }
      else {
        this.lx = 0.0;
      }
      if (initObj.hasOwnProperty('ly')) {
        this.ly = initObj.ly
      }
      else {
        this.ly = 0.0;
      }
      if (initObj.hasOwnProperty('lyaw')) {
        this.lyaw = initObj.lyaw
      }
      else {
        this.lyaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointTraj
    // Serialize message field [t]
    bufferOffset = _serializer.int32(obj.t, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float32(obj.d, buffer, bufferOffset);
    // Serialize message field [d_d]
    bufferOffset = _serializer.float32(obj.d_d, buffer, bufferOffset);
    // Serialize message field [d_dd]
    bufferOffset = _serializer.float32(obj.d_dd, buffer, bufferOffset);
    // Serialize message field [d_ddd]
    bufferOffset = _serializer.float32(obj.d_ddd, buffer, bufferOffset);
    // Serialize message field [s]
    bufferOffset = _serializer.float32(obj.s, buffer, bufferOffset);
    // Serialize message field [s_d]
    bufferOffset = _serializer.float32(obj.s_d, buffer, bufferOffset);
    // Serialize message field [s_dd]
    bufferOffset = _serializer.float32(obj.s_dd, buffer, bufferOffset);
    // Serialize message field [s_ddd]
    bufferOffset = _serializer.float32(obj.s_ddd, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [ks]
    bufferOffset = _serializer.float32(obj.ks, buffer, bufferOffset);
    // Serialize message field [lx]
    bufferOffset = _serializer.float32(obj.lx, buffer, bufferOffset);
    // Serialize message field [ly]
    bufferOffset = _serializer.float32(obj.ly, buffer, bufferOffset);
    // Serialize message field [lyaw]
    bufferOffset = _serializer.float32(obj.lyaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointTraj
    let len;
    let data = new PointTraj(null);
    // Deserialize message field [t]
    data.t = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [d_d]
    data.d_d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [d_dd]
    data.d_dd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [d_ddd]
    data.d_ddd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [s]
    data.s = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [s_d]
    data.s_d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [s_dd]
    data.s_dd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [s_ddd]
    data.s_ddd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ks]
    data.ks = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lx]
    data.lx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ly]
    data.ly = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lyaw]
    data.lyaw = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plan_msgs/PointTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '819fa2e7955aef10d3df960cc6b9813c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 t
    float32 d
    float32 d_d
    float32 d_dd
    float32 d_ddd
    float32 s
    float32 s_d
    float32 s_dd
    float32 s_ddd
    float32 x
    float32 y
    float32 yaw
    float32 ks
    float32 lx
    float32 ly
    float32 lyaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointTraj(null);
    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = 0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.d_d !== undefined) {
      resolved.d_d = msg.d_d;
    }
    else {
      resolved.d_d = 0.0
    }

    if (msg.d_dd !== undefined) {
      resolved.d_dd = msg.d_dd;
    }
    else {
      resolved.d_dd = 0.0
    }

    if (msg.d_ddd !== undefined) {
      resolved.d_ddd = msg.d_ddd;
    }
    else {
      resolved.d_ddd = 0.0
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = 0.0
    }

    if (msg.s_d !== undefined) {
      resolved.s_d = msg.s_d;
    }
    else {
      resolved.s_d = 0.0
    }

    if (msg.s_dd !== undefined) {
      resolved.s_dd = msg.s_dd;
    }
    else {
      resolved.s_dd = 0.0
    }

    if (msg.s_ddd !== undefined) {
      resolved.s_ddd = msg.s_ddd;
    }
    else {
      resolved.s_ddd = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.ks !== undefined) {
      resolved.ks = msg.ks;
    }
    else {
      resolved.ks = 0.0
    }

    if (msg.lx !== undefined) {
      resolved.lx = msg.lx;
    }
    else {
      resolved.lx = 0.0
    }

    if (msg.ly !== undefined) {
      resolved.ly = msg.ly;
    }
    else {
      resolved.ly = 0.0
    }

    if (msg.lyaw !== undefined) {
      resolved.lyaw = msg.lyaw;
    }
    else {
      resolved.lyaw = 0.0
    }

    return resolved;
    }
};

module.exports = PointTraj;
