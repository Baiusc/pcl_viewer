#ifndef __RX_DATA_SERVICE_H__
#define __RX_DATA_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "rx_cfg_loader.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t vc0_w;          ///< Width of virtual channel 0 in pixels
    uint32_t vc0_h;          ///< Height of virtual channel 0 in pixels
    uint32_t vc1_w;          ///< Width of virtual channel 1 in pixels
    uint32_t vc1_h;          ///< Height of virtual channel 1 in pixels
    bool is_doubel_vc;       ///< Flag indicating dual virtual channel mode
} rxdata_cfg_t;

/**
 * @brief Initialize the RX data service
 *
 * This function performs initialization operations for the receive data service,
 * setting up necessary resources and configurations required for data reception.
 *
 * @return int Returns 0 on success, or a negative error code on failure
 */
int rxdata_service_init(const rx_parse_cfg_t *rx_parse);

/**
 * @brief Deinitialize the RX data service
 *
 * This function performs cleanup and deinitialization of the RX data service,
 * releasing any allocated resources and stopping any ongoing operations.
 *
 * @return int Returns 0 on success, negative error code on failure
 */
int rxdata_service_deinit(void);

/**
 * @brief Stop the RX data service without clearing configuration
 *
 * This stops streaming threads and releases CSI resources so that
 * the service can be restarted quickly with existing configuration.
 *
 * @return int 0 on success, negative on failure
 */
int rxdata_service_stop(void);

/**
 * @brief Cleanup CSI resources after algorithm threads have stopped
 *
 * This function deinitializes CSI hardware and frees related buffers.
 * MUST be called AFTER algo_deinit() to ensure no threads are accessing
 * CSI buffers during cleanup.
 *
 * @return int 0 on success, negative on failure
 */
int rxdata_service_cleanup_csi(void);

/**
 * @brief Restart the RX data service with current configuration
 *
 * Convenience wrapper: stop then init. Keeps the cached configuration
 * and reinitializes streaming threads and CSI.
 *
 * @return int 0 on success, negative on failure
 */
int rxdata_service_restart(const rx_parse_cfg_t *rx_parse);

/**
 * @brief Releases a CSI frame buffer back to the service
 *
 * This function returns a previously acquired CSI frame buffer to the
 * rxdata service, making it available for reuse. It should be called
 * after the application has finished processing the frame data.
 *
 * @param[in] frame Pointer to the CSI buffer structure to be released.
 *                  Must be a valid pointer to a previously acquired frame.
 *
 * @return int Status code indicating the result of the operation
 *             - 0: Success, frame released successfully
 *             - Non-zero: Error code indicating failure reason
 *
 * @note The caller should not access the frame buffer after calling this function
 * @note Passing a NULL pointer or invalid frame may result in undefined behavior
 */
int rxdata_service_release_frame(void* frame);

/**
 * @brief Sets the configuration for the RX data service
 *
 * This function configures the receive data service with the specified parameters
 * for single or dual virtual channel mode.
 *
 * @param is_double_vc Flag indicating whether to use dual virtual channel mode
 *                     - true: Enable dual VC mode (VC0 and VC1)
 *                     - false: Single VC mode (VC0 only)
 * @param vc0_w Width of virtual channel 0 in pixels
 * @param vc0_h Height of virtual channel 0 in pixels
 * @param vc1_w Width of virtual channel 1 in pixels (ignored if is_double_vc is false)
 * @param vc1_h Height of virtual channel 1 in pixels (ignored if is_double_vc is false)
 *
 * @return Status code indicating success or failure
 *         - FS_SUCCESS: Configuration successful
 *         - Error code: Configuration failed
 *
 * @note This function must be called before rxdata_service_init()
 * @note Configuration is stored globally and used during service initialization
 */
int rxdata_service_set_cfg(bool is_double_vc, uint32_t vc0_w, uint32_t vc0_h, uint32_t vc1_w, uint32_t vc1_h);

#ifdef __cplusplus
}
#endif

#endif // __RX_DATA_SERVICE_H__
