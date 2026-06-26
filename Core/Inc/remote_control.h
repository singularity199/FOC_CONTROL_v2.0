#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include <stdint.h>

void remote_control_init(void);
void remote_control_rx_feed(uint8_t *data, uint32_t len);
void remote_control_task(void);

#endif
