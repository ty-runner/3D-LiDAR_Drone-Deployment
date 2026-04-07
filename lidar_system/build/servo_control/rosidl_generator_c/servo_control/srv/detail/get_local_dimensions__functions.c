// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice
#include "servo_control/srv/detail/get_local_dimensions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
servo_control__srv__GetLocalDimensions_Request__init(servo_control__srv__GetLocalDimensions_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  return true;
}

void
servo_control__srv__GetLocalDimensions_Request__fini(servo_control__srv__GetLocalDimensions_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
}

bool
servo_control__srv__GetLocalDimensions_Request__are_equal(const servo_control__srv__GetLocalDimensions_Request * lhs, const servo_control__srv__GetLocalDimensions_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
servo_control__srv__GetLocalDimensions_Request__copy(
  const servo_control__srv__GetLocalDimensions_Request * input,
  servo_control__srv__GetLocalDimensions_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

servo_control__srv__GetLocalDimensions_Request *
servo_control__srv__GetLocalDimensions_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_control__srv__GetLocalDimensions_Request * msg = (servo_control__srv__GetLocalDimensions_Request *)allocator.allocate(sizeof(servo_control__srv__GetLocalDimensions_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(servo_control__srv__GetLocalDimensions_Request));
  bool success = servo_control__srv__GetLocalDimensions_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
servo_control__srv__GetLocalDimensions_Request__destroy(servo_control__srv__GetLocalDimensions_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    servo_control__srv__GetLocalDimensions_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
servo_control__srv__GetLocalDimensions_Request__Sequence__init(servo_control__srv__GetLocalDimensions_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_control__srv__GetLocalDimensions_Request * data = NULL;

  if (size) {
    data = (servo_control__srv__GetLocalDimensions_Request *)allocator.zero_allocate(size, sizeof(servo_control__srv__GetLocalDimensions_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = servo_control__srv__GetLocalDimensions_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        servo_control__srv__GetLocalDimensions_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
servo_control__srv__GetLocalDimensions_Request__Sequence__fini(servo_control__srv__GetLocalDimensions_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      servo_control__srv__GetLocalDimensions_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

servo_control__srv__GetLocalDimensions_Request__Sequence *
servo_control__srv__GetLocalDimensions_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_control__srv__GetLocalDimensions_Request__Sequence * array = (servo_control__srv__GetLocalDimensions_Request__Sequence *)allocator.allocate(sizeof(servo_control__srv__GetLocalDimensions_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = servo_control__srv__GetLocalDimensions_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
servo_control__srv__GetLocalDimensions_Request__Sequence__destroy(servo_control__srv__GetLocalDimensions_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    servo_control__srv__GetLocalDimensions_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
servo_control__srv__GetLocalDimensions_Request__Sequence__are_equal(const servo_control__srv__GetLocalDimensions_Request__Sequence * lhs, const servo_control__srv__GetLocalDimensions_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!servo_control__srv__GetLocalDimensions_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
servo_control__srv__GetLocalDimensions_Request__Sequence__copy(
  const servo_control__srv__GetLocalDimensions_Request__Sequence * input,
  servo_control__srv__GetLocalDimensions_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(servo_control__srv__GetLocalDimensions_Request);
    servo_control__srv__GetLocalDimensions_Request * data =
      (servo_control__srv__GetLocalDimensions_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!servo_control__srv__GetLocalDimensions_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          servo_control__srv__GetLocalDimensions_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!servo_control__srv__GetLocalDimensions_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
servo_control__srv__GetLocalDimensions_Response__init(servo_control__srv__GetLocalDimensions_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    servo_control__srv__GetLocalDimensions_Response__fini(msg);
    return false;
  }
  // floor_z
  // ceiling_z
  // room_height
  // clearance_below
  // clearance_above
  return true;
}

void
servo_control__srv__GetLocalDimensions_Response__fini(servo_control__srv__GetLocalDimensions_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // floor_z
  // ceiling_z
  // room_height
  // clearance_below
  // clearance_above
}

bool
servo_control__srv__GetLocalDimensions_Response__are_equal(const servo_control__srv__GetLocalDimensions_Response * lhs, const servo_control__srv__GetLocalDimensions_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // floor_z
  if (lhs->floor_z != rhs->floor_z) {
    return false;
  }
  // ceiling_z
  if (lhs->ceiling_z != rhs->ceiling_z) {
    return false;
  }
  // room_height
  if (lhs->room_height != rhs->room_height) {
    return false;
  }
  // clearance_below
  if (lhs->clearance_below != rhs->clearance_below) {
    return false;
  }
  // clearance_above
  if (lhs->clearance_above != rhs->clearance_above) {
    return false;
  }
  return true;
}

bool
servo_control__srv__GetLocalDimensions_Response__copy(
  const servo_control__srv__GetLocalDimensions_Response * input,
  servo_control__srv__GetLocalDimensions_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // floor_z
  output->floor_z = input->floor_z;
  // ceiling_z
  output->ceiling_z = input->ceiling_z;
  // room_height
  output->room_height = input->room_height;
  // clearance_below
  output->clearance_below = input->clearance_below;
  // clearance_above
  output->clearance_above = input->clearance_above;
  return true;
}

servo_control__srv__GetLocalDimensions_Response *
servo_control__srv__GetLocalDimensions_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_control__srv__GetLocalDimensions_Response * msg = (servo_control__srv__GetLocalDimensions_Response *)allocator.allocate(sizeof(servo_control__srv__GetLocalDimensions_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(servo_control__srv__GetLocalDimensions_Response));
  bool success = servo_control__srv__GetLocalDimensions_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
servo_control__srv__GetLocalDimensions_Response__destroy(servo_control__srv__GetLocalDimensions_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    servo_control__srv__GetLocalDimensions_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
servo_control__srv__GetLocalDimensions_Response__Sequence__init(servo_control__srv__GetLocalDimensions_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_control__srv__GetLocalDimensions_Response * data = NULL;

  if (size) {
    data = (servo_control__srv__GetLocalDimensions_Response *)allocator.zero_allocate(size, sizeof(servo_control__srv__GetLocalDimensions_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = servo_control__srv__GetLocalDimensions_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        servo_control__srv__GetLocalDimensions_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
servo_control__srv__GetLocalDimensions_Response__Sequence__fini(servo_control__srv__GetLocalDimensions_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      servo_control__srv__GetLocalDimensions_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

servo_control__srv__GetLocalDimensions_Response__Sequence *
servo_control__srv__GetLocalDimensions_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_control__srv__GetLocalDimensions_Response__Sequence * array = (servo_control__srv__GetLocalDimensions_Response__Sequence *)allocator.allocate(sizeof(servo_control__srv__GetLocalDimensions_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = servo_control__srv__GetLocalDimensions_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
servo_control__srv__GetLocalDimensions_Response__Sequence__destroy(servo_control__srv__GetLocalDimensions_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    servo_control__srv__GetLocalDimensions_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
servo_control__srv__GetLocalDimensions_Response__Sequence__are_equal(const servo_control__srv__GetLocalDimensions_Response__Sequence * lhs, const servo_control__srv__GetLocalDimensions_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!servo_control__srv__GetLocalDimensions_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
servo_control__srv__GetLocalDimensions_Response__Sequence__copy(
  const servo_control__srv__GetLocalDimensions_Response__Sequence * input,
  servo_control__srv__GetLocalDimensions_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(servo_control__srv__GetLocalDimensions_Response);
    servo_control__srv__GetLocalDimensions_Response * data =
      (servo_control__srv__GetLocalDimensions_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!servo_control__srv__GetLocalDimensions_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          servo_control__srv__GetLocalDimensions_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!servo_control__srv__GetLocalDimensions_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
