



#define SRC_SOAR_FIN_FIN_SERVER_H_
/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */
/**
   @file
   Local fin controller server API
*/

#include <fin.h>



gs_fin_cmd_error_t init_server(void);
/**
   Get fin status
   Updates status struct with content.
   @note data must be in network byte order.
   @note this function should be implemented by UNIMAN.
   @param[out] status Status struct in network byte order.
   @returns cmd error code, see @gs_fin_cmd_error_t
 */
gs_fin_cmd_error_t get_fin_status(gs_fin_status_t * status);

/**
   Set fin positions.
   @note this function should be implemented by UNIMAN.
   @param[in] pos Set points struct.
   @returns cmd error code, see @gs_fin_cmd_error_t
 */
gs_fin_cmd_error_t set_fin_pos(const gs_fin_positions_t * pos);

/**
   Get fin configuration
   Updates config struct with content.
   @note data must be in network byte order.
   @note this function should be implemented by UNIMAN.
   @param[out] conf Config struct in network byte order.
   @returns cmd error code, see @gs_fin_cmd_error_t
*/
gs_fin_cmd_error_t get_fin_config(gs_fin_config_t * conf);

/**
   Set fin configuration.
   @note this function should be implemented by UNIMAN.
   @param[in] conf Config struct.
   @returns cmd error code, see @gs_fin_cmd_error_t
 */
gs_fin_cmd_error_t set_fin_config(const gs_fin_config_t * conf);

/**
   Save fin configuration.
   @note this function should be implemented by UNIMAN.
   @returns cmd error code, see @gs_fin_cmd_error_t
 */
gs_fin_cmd_error_t save_fin_config(void);


/**
Internal only load command to restore running config from eeprom
*/
gs_fin_cmd_error_t load_fin_config(void);




