/** @file
 *  @brief ESS Service
 */

/*
 */

#ifdef __cplusplus
extern "C" {
#endif

void ess_init(void);
int  ess_notify_temp(s16_t temperature);
int  ess_notify_humi(u16_t humidity);

#ifdef __cplusplus
}
#endif
