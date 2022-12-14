# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cloud_msgs/Grid.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import cloud_msgs.msg
import genpy

class Grid(genpy.Message):
  _md5sum = "ec789739e2a01936ea531728fbd248c4"
  _type = "cloud_msgs/Grid"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time       timestamp
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
"""
  __slots__ = ['timestamp','width','height','width_step','height_step','grid_nums','grid','enabled','pos_vehicle']
  _slot_types = ['time','int32','int32','float32','float32','int32','int8[]','int8','cloud_msgs/PointXYA']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,width,height,width_step,height_step,grid_nums,grid,enabled,pos_vehicle

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Grid, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.width is None:
        self.width = 0
      if self.height is None:
        self.height = 0
      if self.width_step is None:
        self.width_step = 0.
      if self.height_step is None:
        self.height_step = 0.
      if self.grid_nums is None:
        self.grid_nums = 0
      if self.grid is None:
        self.grid = []
      if self.enabled is None:
        self.enabled = 0
      if self.pos_vehicle is None:
        self.pos_vehicle = cloud_msgs.msg.PointXYA()
    else:
      self.timestamp = genpy.Time()
      self.width = 0
      self.height = 0
      self.width_step = 0.
      self.height_step = 0.
      self.grid_nums = 0
      self.grid = []
      self.enabled = 0
      self.pos_vehicle = cloud_msgs.msg.PointXYA()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2I2i2fi().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.width, _x.height, _x.width_step, _x.height_step, _x.grid_nums))
      length = len(self.grid)
      buff.write(_struct_I.pack(length))
      pattern = '<%sb'%length
      buff.write(struct.pack(pattern, *self.grid))
      _x = self
      buff.write(_get_struct_b3d().pack(_x.enabled, _x.pos_vehicle.x, _x.pos_vehicle.y, _x.pos_vehicle.yaw))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.pos_vehicle is None:
        self.pos_vehicle = cloud_msgs.msg.PointXYA()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.width, _x.height, _x.width_step, _x.height_step, _x.grid_nums,) = _get_struct_2I2i2fi().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sb'%length
      start = end
      end += struct.calcsize(pattern)
      self.grid = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 25
      (_x.enabled, _x.pos_vehicle.x, _x.pos_vehicle.y, _x.pos_vehicle.yaw,) = _get_struct_b3d().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2I2i2fi().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.width, _x.height, _x.width_step, _x.height_step, _x.grid_nums))
      length = len(self.grid)
      buff.write(_struct_I.pack(length))
      pattern = '<%sb'%length
      buff.write(self.grid.tostring())
      _x = self
      buff.write(_get_struct_b3d().pack(_x.enabled, _x.pos_vehicle.x, _x.pos_vehicle.y, _x.pos_vehicle.yaw))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.pos_vehicle is None:
        self.pos_vehicle = cloud_msgs.msg.PointXYA()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.width, _x.height, _x.width_step, _x.height_step, _x.grid_nums,) = _get_struct_2I2i2fi().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sb'%length
      start = end
      end += struct.calcsize(pattern)
      self.grid = numpy.frombuffer(str[start:end], dtype=numpy.int8, count=length)
      _x = self
      start = end
      end += 25
      (_x.enabled, _x.pos_vehicle.x, _x.pos_vehicle.y, _x.pos_vehicle.yaw,) = _get_struct_b3d().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I2i2fi = None
def _get_struct_2I2i2fi():
    global _struct_2I2i2fi
    if _struct_2I2i2fi is None:
        _struct_2I2i2fi = struct.Struct("<2I2i2fi")
    return _struct_2I2i2fi
_struct_b3d = None
def _get_struct_b3d():
    global _struct_b3d
    if _struct_b3d is None:
        _struct_b3d = struct.Struct("<b3d")
    return _struct_b3d
