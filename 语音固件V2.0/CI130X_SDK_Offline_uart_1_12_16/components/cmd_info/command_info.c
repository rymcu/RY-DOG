#include "command_info.h"


#if (COMMAND_INFO_VER == 2)

#include "command_info_v2.c"

#elif (COMMAND_INFO_VER == 3)

#include "command_info_v3.c"

#endif
