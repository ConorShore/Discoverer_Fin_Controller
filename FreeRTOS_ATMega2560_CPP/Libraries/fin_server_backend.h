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
   Set maximum drag positions of fins.

   @note this function should be implemented by UNIMAN.

   @returns cmd error code, see @gs_fin_cmd_error_t
 */
gs_fin_cmd_error_t set_max_drag(void);

/**
   Set minimum drag positions of fins.

   @note this function should be implemented by UNIMAN.

   @returns cmd error code, see @gs_fin_cmd_error_t
 */
gs_fin_cmd_error_t set_min_drag(void);

