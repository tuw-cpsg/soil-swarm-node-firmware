/** @file
 *  @brief ESS Service
 */

/*
 */

#ifdef __cplusplus
extern "C" {
#endif

void ess_init(void);
void ess_notify(int16_t temperature, int16_t humidity);

#ifdef __cplusplus
}
#endif
