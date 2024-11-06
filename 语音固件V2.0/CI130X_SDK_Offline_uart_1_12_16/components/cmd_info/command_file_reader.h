#include "sdk_default_config.h"


#define COMMAND_INFO_VER        2


#if (COMMAND_INFO_VER == 2)

#include "command_file_reader_v2.h"

#elif (COMMAND_INFO_VER == 3)

#include "command_file_reader_v3.h"

#endif
