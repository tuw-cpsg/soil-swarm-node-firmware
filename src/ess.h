/** @file
 *  @brief ESS Service
 */

/*
 */

#ifdef __cplusplus
extern "C" {
#endif

void ess_init(void);
int  ess_notify(s16_t temperature, u16_t humidity);

#ifdef __cplusplus
}
#endif
