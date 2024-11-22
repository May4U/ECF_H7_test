#pragma once

#ifndef UTIL_ASSERT_HPP
#define UTIL_ASSERT_HPP

namespace Util
{

#ifndef NO_ASSERT
    inline void Assert(bool expression)
    {
        if (!expression) [[unlikely]]
        {
            while (true)
            {
                ;
            }
        }
    }
#else
    inline void Assert(bool expression) {}
#endif


}// namespace Util

#endif//UTIL_ASSERT_HPP
