#include <kernel.h>
#include <kernel_internal.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <arch/cpu.h>
#include <logging/log_ctrl.h>
#include <logging/log.h>
#include <fatal.h>
#include <misc/reboot.h>

//LOG_MODULE_DECLARE(os);

/**
 *
 * @brief Fatal error handler
 *
 * This routine implements the corrective action to be taken when the system
 * detects a fatal error.
 *
 * This sample implementation attempts to abort the current thread and allow
 * the system to continue executing, which may permit the system to continue
 * functioning with degraded capabilities.
 *
 * System designers may wish to enhance or substitute this sample
 * implementation to take other actions, such as logging error (or debug)
 * information to a persistent repository and/or rebooting the system.
 *
 * @param reason fatal error reason
 * @param pEsf pointer to exception stack frame
 *
 * @return This function does not return.
 */
void __weak k_sys_fatal_error_handler(unsigned int reason,
	      const z_arch_esf_t *esf)
{
	ARG_UNUSED(esf);

	LOG_PANIC();
	z_fatal_print("restarting system");

    sys_reboot(0);
    
    CODE_UNREACHABLE;
}
