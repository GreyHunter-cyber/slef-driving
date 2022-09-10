// Auto-generated. Do not edit!

// (in-package cloud_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PointXYA = require('./PointXYA.js');

//-----------------------------------------------------------

class Grid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.width = null;
      this.height = null;
      this.width_step = null;
      this.height_step = null;
      this.grid_nums = null;
      this.grid = null;
      this.enabled = null;
      this.pos_vehicle = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('width_step')) {
        this.width_step = initObj.width_step
      }
      else {
        this.width_step = 0.0;
      }
      if (initObj.hasOwnProperty('height_step')) {
        this.height_step = initObj.height_step
      }
      else {
        this.height_step = 0.0;
      }
      if (initObj.hasOwnProperty('grid_nums')) {
        this.grid_nums = initObj.grid_nums
      }
      else {
        this.grid_nums = 0;
      }
      if (initObj.hasOwnProperty('grid')) {
        this.grid = initObj.grid
      }
      else {
        this.grid = [];
      }
      if (initObj.hasOwnProperty('enabled')) {
        this.enabled = initObj.enabled
      }
      else {
        this.enabled = 0;
      }
      if (initObj.hasOwnProperty('pos_vehicle')) {
        this.pos_vehicle = initObj.pos_vehicle
      }
      else {
        this.pos_vehicle = new PointXYA();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Grid
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.int32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int32(obj.height, buffer, bufferOffset);
    // Serialize message field [width_step]
    bufferOffset = _serializer.float32(obj.width_step, buffer, bufferOffset);
    // Serialize message field [height_step]
    bufferOffset = _serializer.float32(obj.height_step, buffer, bufferOffset);
    // Serialize message field [grid_nums]
    bufferOffset = _serializer.int32(obj.grid_nums, buffer, bufferOffset);
    // Serialize message field [grid]
    bufferOffset = _arraySerializer.int8(obj.grid, buffer, bufferOffset, null);
    // Serialize message field [enabled]
    bufferOffset = _serializer.int8(obj.enabled, buffer, bufferOffset);
    // Serialize message field [pos_vehicle]
    bufferOffset = PointXYA.serialize(obj.pos_vehicle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Grid
    let len;
    let data = new Grid(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [width_step]
    data.width_step = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height_step]
    data.height_step = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grid_nums]
    data.grid_nums = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [grid]
    data.grid = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [enabled]
    data.enabled = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [pos_vehicle]
    data.pos_vehicle = PointXYA.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.grid.length;
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cloud_msgs/Grid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec789739e2a01936ea531728fbd248c4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time       timestamp
    int32      width
    int32      height
    float32    width_step
    float32    height_step
    int32      grid_nums
    int8[]     grid
    int8       enabled
    PointXYA   pos_vehicle
    
    ================================================================================
    MSG: cloud_msgs/PointXYA
    float64 x
    float64 y
    float64 yaw  # degs
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Grid(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.width_step !== undefined) {
      resolved.width_step = msg.width_step;
    }
    else {
      resolved.width_step = 0.0
    }

    if (msg.height_step !== undefined) {
      resolved.height_step = msg.height_step;
    }
    else {
      resolved.height_step = 0.0
    }

    if (msg.grid_nums !== undefined) {
      resolved.grid_nums = msg.grid_nums;
    }
    else {
      resolved.grid_nums = 0
    }

    if (msg.grid !== undefined) {
      resolved.grid = msg.grid;
    }
    else {
      resolved.grid = []
    }

    if (msg.enabled !== undefined) {
      resolved.enabled = msg.enabled;
    }
    else {
      resolved.enabled = 0
    }

    if (msg.pos_vehicle !== undefined) {
      resolved.pos_vehicle = PointXYA.Resolve(msg.pos_vehicle)
    }
    else {
      resolved.pos_vehicle = new PointXYA()
    }

    return resolved;
    }
};

module.exports = Grid;
