#pragma once

#ifndef UTIL_INSTANCE_MANAGER_HPP
#define UTIL_INSTANCE_MANAGER_HPP

#include "util_assert.hpp"
#include <cstdint>

namespace Util
{

    /**
     * 实例管理器
     * @tparam T 类
     * @tparam capacity 最大容量
     */
    template<class T, uint8_t capacity>
    class InstanceManger
    {
    private:
        T *data_[capacity];
        uint8_t size_ = 0;

    public:
        void add(T *t)
        {
            Assert(size_ < capacity);
            data_[++size_] = t;
        }

        [[nodiscard]] T *get(const uint8_t &index)
        {
            Assert(index < size_);
            return data_[index];
        }

        [[nodiscard]] uint8_t size() const
        {
            return size_;
        }
    };

}// namespace Util

#endif//UTIL_INSTANCE_MANAGER_HPP
