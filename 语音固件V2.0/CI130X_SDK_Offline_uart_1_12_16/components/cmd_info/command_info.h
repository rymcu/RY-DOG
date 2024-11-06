#include "sdk_default_config.h"
#include "command_file_reader.h"




#if (COMMAND_INFO_VER == 2)

#include "command_info_v2.h"

#elif (COMMAND_INFO_VER == 3)

#include "command_info_v3.h"

#endif
