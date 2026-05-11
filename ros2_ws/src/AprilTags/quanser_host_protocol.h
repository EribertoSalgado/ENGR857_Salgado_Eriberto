#if !defined(_quanser_host_protocol_h)
#define _quanser_host_protocol_h

#include "quanser_types.h"

typedef enum tag_host_command
{
    HOST_COMMAND_INVALID,       /* avoid zero as a command for additional error checking */
    HOST_COMMAND_LOAD,
    HOST_COMMAND_SET_PROPERTY
} t_host_command;

typedef struct tag_host_command_header
{
    t_uint32 peripheral_id;
    t_uint32 command;
} t_host_command_header;

typedef struct tag_host_data_header
{
    t_uint32 peripheral_id;
    t_uint32 data_length;       /* should not exceed 2^31-1 (i.e. INT_MAX) */
} t_host_data_header;

#endif
