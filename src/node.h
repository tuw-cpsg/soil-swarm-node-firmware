#ifndef NODE_H_
#define NODE_H_

#include <zephyr/types.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct measurement {
	u32_t timestamp;
	u16_t battery;
	u16_t moisture;
	s16_t temperature;
};

u8_t node_initialize(void);
void node_start_sensing(s32_t delay);

size_t node_buf_get_elem_size(void);
size_t node_buf_cpy_data(void *dst, size_t cnt);
u8_t   node_buf_get_finish(size_t cnt);

#ifdef __cplusplus
}
#endif

#endif /* NODE_H_ */
