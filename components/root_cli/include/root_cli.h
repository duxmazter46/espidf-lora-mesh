#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void root_cli_start(void);

uint64_t root_get_epoch_ms(void);

#ifdef __cplusplus
}
#endif