#ifndef SRC_SOAR_FIN_FIN_SERVER_H_
#define SRC_SOAR_FIN_FIN_SERVER_H_
/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */
/**
   @file

   Local fin controller server API
   
*/

#include <fin.h>
#include <csp/csp.h>
#include <csp/arch/csp_thread.h>



/**
New func added to access the server task defined in fin_server.c

*/
CSP_DEFINE_TASK(task_server);

gs_fin_cmd_error_t init_server(void);




#endif /* SRC_SOAR_FIN_FIN_SERVER_H_ */
