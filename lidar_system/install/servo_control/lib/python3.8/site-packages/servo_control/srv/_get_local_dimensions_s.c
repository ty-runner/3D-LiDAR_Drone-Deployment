// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "servo_control/srv/detail/get_local_dimensions__struct.h"
#include "servo_control/srv/detail/get_local_dimensions__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool servo_control__srv__get_local_dimensions__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("servo_control.srv._get_local_dimensions.GetLocalDimensions_Request", full_classname_dest, 66) == 0);
  }
  servo_control__srv__GetLocalDimensions_Request * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * servo_control__srv__get_local_dimensions__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetLocalDimensions_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("servo_control.srv._get_local_dimensions");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetLocalDimensions_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  servo_control__srv__GetLocalDimensions_Request * ros_message = (servo_control__srv__GetLocalDimensions_Request *)raw_ros_message;
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "servo_control/srv/detail/get_local_dimensions__struct.h"
// already included above
// #include "servo_control/srv/detail/get_local_dimensions__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool servo_control__srv__get_local_dimensions__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("servo_control.srv._get_local_dimensions.GetLocalDimensions_Response", full_classname_dest, 67) == 0);
  }
  servo_control__srv__GetLocalDimensions_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // floor_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "floor_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->floor_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ceiling_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "ceiling_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ceiling_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // room_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "room_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->room_height = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // clearance_below
    PyObject * field = PyObject_GetAttrString(_pymsg, "clearance_below");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->clearance_below = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // clearance_above
    PyObject * field = PyObject_GetAttrString(_pymsg, "clearance_above");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->clearance_above = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * servo_control__srv__get_local_dimensions__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetLocalDimensions_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("servo_control.srv._get_local_dimensions");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetLocalDimensions_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  servo_control__srv__GetLocalDimensions_Response * ros_message = (servo_control__srv__GetLocalDimensions_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // floor_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->floor_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "floor_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ceiling_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ceiling_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ceiling_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // room_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->room_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "room_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // clearance_below
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->clearance_below);
    {
      int rc = PyObject_SetAttrString(_pymessage, "clearance_below", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // clearance_above
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->clearance_above);
    {
      int rc = PyObject_SetAttrString(_pymessage, "clearance_above", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
