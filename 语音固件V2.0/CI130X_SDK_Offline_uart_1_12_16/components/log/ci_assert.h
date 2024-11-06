/**
 * @file ci_assert.h
 * @brief assert
 * @version 1.0
 * @date 2019-02-21
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _CI_ASSERT_H_
#define _CI_ASSERT_H_

#include "ci_log.h"
#include "ci_debug_config.h"

#if CONFIG_DEBUG_EN
#define CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        ci_logerr(LOG_ASSERT,"%s",msg);                                                                                     \
        ci_logassert(LOG_ASSERT,"ASSERT: %d in %s\n",__LINE__,__FUNCTION__);                                                \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }
#else
#define CI_ASSERT(x,msg) do{}while(0)
#endif

#endif /* _CI_ASSERT_H_ */
