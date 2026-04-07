# generated from rosidl_generator_py/resource/_idl.py.em
# with input from servo_control:srv/GetLocalDimensions.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetLocalDimensions_Request(type):
    """Metaclass of message 'GetLocalDimensions_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('servo_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'servo_control.srv.GetLocalDimensions_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_local_dimensions__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_local_dimensions__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_local_dimensions__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_local_dimensions__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_local_dimensions__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetLocalDimensions_Request(metaclass=Metaclass_GetLocalDimensions_Request):
    """Message class 'GetLocalDimensions_Request'."""

    __slots__ = [
        '_x',
        '_y',
        '_z',
    ]

    _fields_and_field_types = {
        'x': 'double',
        'y': 'double',
        'z': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
        self._z = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_GetLocalDimensions_Response(type):
    """Metaclass of message 'GetLocalDimensions_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('servo_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'servo_control.srv.GetLocalDimensions_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_local_dimensions__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_local_dimensions__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_local_dimensions__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_local_dimensions__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_local_dimensions__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetLocalDimensions_Response(metaclass=Metaclass_GetLocalDimensions_Response):
    """Message class 'GetLocalDimensions_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_floor_z',
        '_ceiling_z',
        '_room_height',
        '_clearance_below',
        '_clearance_above',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'floor_z': 'double',
        'ceiling_z': 'double',
        'room_height': 'double',
        'clearance_below': 'double',
        'clearance_above': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.floor_z = kwargs.get('floor_z', float())
        self.ceiling_z = kwargs.get('ceiling_z', float())
        self.room_height = kwargs.get('room_height', float())
        self.clearance_below = kwargs.get('clearance_below', float())
        self.clearance_above = kwargs.get('clearance_above', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        if self.floor_z != other.floor_z:
            return False
        if self.ceiling_z != other.ceiling_z:
            return False
        if self.room_height != other.room_height:
            return False
        if self.clearance_below != other.clearance_below:
            return False
        if self.clearance_above != other.clearance_above:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @property
    def floor_z(self):
        """Message field 'floor_z'."""
        return self._floor_z

    @floor_z.setter
    def floor_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'floor_z' field must be of type 'float'"
        self._floor_z = value

    @property
    def ceiling_z(self):
        """Message field 'ceiling_z'."""
        return self._ceiling_z

    @ceiling_z.setter
    def ceiling_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ceiling_z' field must be of type 'float'"
        self._ceiling_z = value

    @property
    def room_height(self):
        """Message field 'room_height'."""
        return self._room_height

    @room_height.setter
    def room_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'room_height' field must be of type 'float'"
        self._room_height = value

    @property
    def clearance_below(self):
        """Message field 'clearance_below'."""
        return self._clearance_below

    @clearance_below.setter
    def clearance_below(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'clearance_below' field must be of type 'float'"
        self._clearance_below = value

    @property
    def clearance_above(self):
        """Message field 'clearance_above'."""
        return self._clearance_above

    @clearance_above.setter
    def clearance_above(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'clearance_above' field must be of type 'float'"
        self._clearance_above = value


class Metaclass_GetLocalDimensions(type):
    """Metaclass of service 'GetLocalDimensions'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('servo_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'servo_control.srv.GetLocalDimensions')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_local_dimensions

            from servo_control.srv import _get_local_dimensions
            if _get_local_dimensions.Metaclass_GetLocalDimensions_Request._TYPE_SUPPORT is None:
                _get_local_dimensions.Metaclass_GetLocalDimensions_Request.__import_type_support__()
            if _get_local_dimensions.Metaclass_GetLocalDimensions_Response._TYPE_SUPPORT is None:
                _get_local_dimensions.Metaclass_GetLocalDimensions_Response.__import_type_support__()


class GetLocalDimensions(metaclass=Metaclass_GetLocalDimensions):
    from servo_control.srv._get_local_dimensions import GetLocalDimensions_Request as Request
    from servo_control.srv._get_local_dimensions import GetLocalDimensions_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
