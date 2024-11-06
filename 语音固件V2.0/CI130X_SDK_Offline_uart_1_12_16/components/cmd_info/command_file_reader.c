#include "sdk_default_config.h"
#include "command_file_reader.h"

#if (COMMAND_INFO_VER == 2)

#include "command_file_reader_v2.c"

#elif (COMMAND_INFO_VER == 3)

#include "command_file_reader_v3.c"

#endif
