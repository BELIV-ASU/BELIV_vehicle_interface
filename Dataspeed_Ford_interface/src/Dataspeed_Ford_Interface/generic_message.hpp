/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <cstring>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

namespace ros2_generic_message {

using namespace rosidl_typesupport_introspection_cpp;

static bool _is_copy_compatible(const MessageMember *dst, const MessageMember *src) {
  return dst->type_id_ == src->type_id_
      && dst->is_array_ == src->is_array_
      && dst->array_size_ == src->array_size_
      && dst->is_upper_bound_ == src->is_upper_bound_
      && dst->string_upper_bound_ == src->string_upper_bound_;
}

class Msg {
public:
  class MsgField {
  public:
    MsgField(const MessageMember *def, void *data)
        : def_(def), data_(reinterpret_cast<uint8_t *>(data) + def->offset_) {
    }
    const std::string_view name() const {
      return std::string_view(def_->name_);
    }
    uint8_t type_id() const {
      return def_->type_id_;
    }
    const void *ptr() const {
      return data_;
    }
    bool set(const MsgField &src) {
      if (!_is_copy_compatible(def_, src.def_)) {
        return false;
      }
      switch (type_id()) {
        case ROS_TYPE_FLOAT:
          return copy_field<float>(data_, src.ptr());
        case ROS_TYPE_DOUBLE:
          return copy_field<double>(data_, src.ptr());
        case ROS_TYPE_LONG_DOUBLE:
          return copy_field<long double>(data_, src.ptr());
        case ROS_TYPE_CHAR:
          return copy_field<char>(data_, src.ptr());
        case ROS_TYPE_WCHAR:
          return copy_field<char16_t>(data_, src.ptr());
        case ROS_TYPE_BOOLEAN:
          return copy_field<bool>(data_, src.ptr());
        case ROS_TYPE_OCTET:
          return copy_field<uint8_t>(data_, src.ptr());
        case ROS_TYPE_UINT8:
          return copy_field<uint8_t>(data_, src.ptr());
        case ROS_TYPE_INT8:
          return copy_field<int8_t>(data_, src.ptr());
        case ROS_TYPE_UINT16:
          return copy_field<uint16_t>(data_, src.ptr());
        case ROS_TYPE_INT16:
          return copy_field<int16_t>(data_, src.ptr());
        case ROS_TYPE_UINT32:
          return copy_field<uint32_t>(data_, src.ptr());
        case ROS_TYPE_INT32:
          return copy_field<int32_t>(data_, src.ptr());
        case ROS_TYPE_UINT64:
          return copy_field<uint64_t>(data_, src.ptr());
        case ROS_TYPE_INT64:
          return copy_field<int64_t>(data_, src.ptr());
        case ROS_TYPE_STRING:
          return copy_field<std::string>(data_, src.ptr());
        case ROS_TYPE_WSTRING:
          return copy_field<std::u16string>(data_, src.ptr());
        case ROS_TYPE_MESSAGE:
          return copy_message(data_, src.def_, src.ptr());
      }
      return false;
    }

  protected:
    template <typename T>
    bool copy_field(void *dst, const void *src) {
      if (!def_->is_array_) {
        *static_cast<T *>(dst) = *static_cast<const T *>(src);
      } else if (def_->array_size_ && !def_->is_upper_bound_) {
        for (size_t index = 0; index < def_->array_size_; ++index) {
          *(static_cast<T *>(dst) + index) = *(static_cast<const T *>(src) + index);
        }
      } else {
        const std::vector<T> &srcVector = *reinterpret_cast<const std::vector<T> *>(src);
        std::vector<T> &dstVector = *reinterpret_cast<std::vector<T> *>(dst);
        std::copy(srcVector.begin(), srcVector.end(), std::back_inserter(dstVector));
      }
      return true;
    }
    bool _copy_message(void *dst, const MessageMember *srcDef, const void *src) {
      return Msg(def_->members_, dst).set(Msg(srcDef->members_, const_cast<void *>(src)));
    }
    bool copy_message(void *dst, const MessageMember *srcDef, const void *src) {
      if (!def_->is_array_) {
        return _copy_message(dst, srcDef, src);
      }
      size_t array_size = 0;
      if (def_->array_size_ && !def_->is_upper_bound_) {
        array_size = def_->array_size_;
      } else {
        array_size = srcDef->size_function(src);
        def_->resize_function(dst, array_size);
      }
      bool result = false;
      for (size_t index = 0; index < array_size; ++index) {
        result |= _copy_message(def_->get_function(dst, index), srcDef, srcDef->get_const_function(src, index));
      }
      return result;
    }

  protected:
    const MessageMember *def_;
    void *data_;

    friend class Msg;
  }; // class MsgField

  Msg(const rosidl_message_type_support_t *def, void *data)
      : Msg(reinterpret_cast<const MessageMembers *>(def->data), data) {
  }
  uint32_t size() const {
    return def_->member_count_;
  }
  const std::string_view ns() const {
    return std::string_view(def_->message_namespace_);
  }
  const std::string_view name() const {
    return std::string_view(def_->message_name_);
  }
  MsgField operator[](const uint32_t index) const {
    if (index >= size()) {
      throw std::out_of_range("Index out of range.");
    }
    return MsgField(def_->members_ + index, data_);
  }
  /**
   * @brief Sets the data of the message with the data of the src message, without knowing the message type. All fields
   * must have the same name and type (including array type and bounds) to be copied. The only exception is for message
   * type names, which may be any value. The limits of bounded strings are considered part of the type for the purposes
   * of determining compatibility.
   *
   * @param src
   * @return true
   * @return false
   */
  bool set(const Msg &src) {
    // we ignore the type of the message and match on field name/type
    bool result = false;
    // caching index pairs could save us time if we need it
    for (uint32_t i = 0; i < src.size(); ++i) {
      for (uint32_t j = 0; j < size(); ++j) {
        // match_message_type
        if (src[i].name() == (*this)[j].name()) {
          result |= (*this)[j].set(src[i]);
        }
      }
    }
    return result;
  }

protected:
  Msg(const MessageMembers *def, void *data) : def_(def), data_(data) {
  }
  const MessageMembers *def_;
  void *data_;

  friend class MsgField;
};

template <typename MsgType>
class Message : public Msg {
public:
  Message(const MsgType *msg) : Msg(get_message_type_support_handle<MsgType>(), (void *)msg) {
  }
};

} // namespace ros2_generic_message
