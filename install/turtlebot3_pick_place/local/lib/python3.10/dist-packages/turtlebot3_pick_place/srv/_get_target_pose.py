# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot3_pick_place:srv/GetTargetPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTargetPose_Request(type):
    """Metaclass of message 'GetTargetPose_Request'."""

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
            module = import_type_support('turtlebot3_pick_place')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_pick_place.srv.GetTargetPose_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_target_pose__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_target_pose__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_target_pose__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_target_pose__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_target_pose__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTargetPose_Request(metaclass=Metaclass_GetTargetPose_Request):
    """Message class 'GetTargetPose_Request'."""

    __slots__ = [
        '_target_class',
    ]

    _fields_and_field_types = {
        'target_class': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target_class = kwargs.get('target_class', str())

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
        if self.target_class != other.target_class:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_class(self):
        """Message field 'target_class'."""
        return self._target_class

    @target_class.setter
    def target_class(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target_class' field must be of type 'str'"
        self._target_class = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTargetPose_Response(type):
    """Metaclass of message 'GetTargetPose_Response'."""

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
            module = import_type_support('turtlebot3_pick_place')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_pick_place.srv.GetTargetPose_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_target_pose__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_target_pose__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_target_pose__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_target_pose__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_target_pose__response

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTargetPose_Response(metaclass=Metaclass_GetTargetPose_Response):
    """Message class 'GetTargetPose_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_target_pose',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'target_pose': 'geometry_msgs/PoseStamped',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        from geometry_msgs.msg import PoseStamped
        self.target_pose = kwargs.get('target_pose', PoseStamped())

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
        if self.target_pose != other.target_pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
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

    @builtins.property
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

    @builtins.property
    def target_pose(self):
        """Message field 'target_pose'."""
        return self._target_pose

    @target_pose.setter
    def target_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'target_pose' field must be a sub message of type 'PoseStamped'"
        self._target_pose = value


class Metaclass_GetTargetPose(type):
    """Metaclass of service 'GetTargetPose'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('turtlebot3_pick_place')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_pick_place.srv.GetTargetPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_target_pose

            from turtlebot3_pick_place.srv import _get_target_pose
            if _get_target_pose.Metaclass_GetTargetPose_Request._TYPE_SUPPORT is None:
                _get_target_pose.Metaclass_GetTargetPose_Request.__import_type_support__()
            if _get_target_pose.Metaclass_GetTargetPose_Response._TYPE_SUPPORT is None:
                _get_target_pose.Metaclass_GetTargetPose_Response.__import_type_support__()


class GetTargetPose(metaclass=Metaclass_GetTargetPose):
    from turtlebot3_pick_place.srv._get_target_pose import GetTargetPose_Request as Request
    from turtlebot3_pick_place.srv._get_target_pose import GetTargetPose_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
