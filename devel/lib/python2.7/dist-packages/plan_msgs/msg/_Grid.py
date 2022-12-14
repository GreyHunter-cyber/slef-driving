# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from plan_msgs/Grid.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class Grid(genpy.Message):
  _md5sum = "84268bd882d6a8d85ea94e3017e923c7"
  _type = "plan_msgs/Grid"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time stamp                                                      #当前时间
int32 width                                                     #宽度分片数，左→右
int32 height                                                    #远近分片数，近→远
int32 size
int32[] value                                                  #路面分片的矩阵
float32 d_width                                                 #格网分辨率
float32 d_height                                                #格网分辨率
"""
  __slots__ = ['stamp','width','height','size','value','d_width','d_height']
  _slot_types = ['time','int32','int32','int32','int32[]','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       stamp,width,height,size,value,d_width,d_height

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Grid, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.stamp is None:
        self.stamp = genpy.Time()
      if self.width is None:
        self.width = 0
      if self.height is None:
        self.height = 0
      if self.size is None:
        self.size = 0
      if self.value is None:
        self.value = []
      if self.d_width is None:
        self.d_width = 0.
      if self.d_height is None:
        self.d_height = 0.
    else:
      self.stamp = genpy.Time()
      self.width = 0
      self.height = 0
      self.size = 0
      self.value = []
      self.d_width = 0.
      self.d_height = 0.

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
      buff.write(_get_struct_2I3i().pack(_x.stamp.secs, _x.stamp.nsecs, _x.width, _x.height, _x.size))
      length = len(self.value)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.value))
      _x = self
      buff.write(_get_struct_2f().pack(_x.d_width, _x.d_height))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.stamp.secs, _x.stamp.nsecs, _x.width, _x.height, _x.size,) = _get_struct_2I3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.value = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 8
      (_x.d_width, _x.d_height,) = _get_struct_2f().unpack(str[start:end])
      self.stamp.canon()
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
      buff.write(_get_struct_2I3i().pack(_x.stamp.secs, _x.stamp.nsecs, _x.width, _x.height, _x.size))
      length = len(self.value)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.value.tostring())
      _x = self
      buff.write(_get_struct_2f().pack(_x.d_width, _x.d_height))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.stamp.secs, _x.stamp.nsecs, _x.width, _x.height, _x.size,) = _get_struct_2I3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.value = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 8
      (_x.d_width, _x.d_height,) = _get_struct_2f().unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I3i = None
def _get_struct_2I3i():
    global _struct_2I3i
    if _struct_2I3i is None:
        _struct_2I3i = struct.Struct("<2I3i")
    return _struct_2I3i
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
