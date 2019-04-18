#include <zephyr.h>

s8_t  onewire_init(void);
void  OWDepower(void);
void  SetSpeed(u8_t standard);
s16_t OWTouchReset(void);
void  OWWriteByte(u8_t data, u8_t power);
u8_t  OWReadByte(void);
u8_t  OWTouchByte(u8_t data);
void  OWBlock(u8_t *data, u16_t data_len);
s16_t OWOverdriveSkip(u8_t *data, u16_t data_len);
