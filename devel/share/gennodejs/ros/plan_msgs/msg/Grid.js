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

class Grid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.width = null;
      this.height = null;
      this.size = null;
      this.value = null;
      this.d_width = null;
      this.d_height = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
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
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = [];
      }
      if (initObj.hasOwnProperty('d_width')) {
        this.d_width = initObj.d_width
      }
      else {
        this.d_width = 0.0;
      }
      if (initObj.hasOwnProperty('d_height')) {
        this.d_height = initObj.d_height
      }
      else {
        this.d_height = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Grid
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.int32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int32(obj.height, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.int32(obj.size, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _arraySerializer.int32(obj.value, buffer, bufferOffset, null);
    // Serialize message field [d_width]
    bufferOffset = _serializer.float32(obj.d_width, buffer, bufferOffset);
    // Serialize message field [d_height]
    bufferOffset = _serializer.float32(obj.d_height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Grid
    let len;
    let data = new Grid(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [d_width]
    data.d_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [d_height]
    data.d_height = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.value.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plan_msgs/Grid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84268bd882d6a8d85ea94e3017e923c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp                                                      #当前时间
    int32 width                                                     #宽度分片数，左→右
    int32 height                                                    #远近分片数，近→远
    int32 size
    int32[] value                                                  #路面分片的矩阵
    float32 d_width                                                 #格网分辨率
    float32 d_height                                                #格网分辨率
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Grid(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
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

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = []
    }

    if (msg.d_width !== undefined) {
      resolved.d_width = msg.d_width;
    }
    else {
      resolved.d_width = 0.0
    }

    if (msg.d_height !== undefined) {
      resolved.d_height = msg.d_height;
    }
    else {
      resolved.d_height = 0.0
    }

    return resolved;
    }
};

module.exports = Grid;
