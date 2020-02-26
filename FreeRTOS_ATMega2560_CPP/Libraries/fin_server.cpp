/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */
/**
   @file

   Fin controller server (implemented on Fin Controller Board)
*/

//csp_thread_handle_t handle_seraver;

#include <stdio.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <csp/arch/csp_thread.h>
#include <fin.h>
#include <fin_server_backend.h>








static void process_fin_cmd(csp_conn_t * conn, csp_packet_t * packet)
{
    uint16_t reply_length = 1;
    uint8_t cmd_id = packet->data[0]; // First byte is command ID

    switch(cmd_id) {
        case GS_FIN_CMD_STATUS: {
            gs_fin_status_t status;

            /* Get status internally */
            int8_t error = get_fin_status(&status);

            /* Set error code in response */
            packet->data[0] = error;

            if (error == FIN_CMD_OK) {
                /* Copy status to response buffer */
                memcpy(&packet->data[1], &status, sizeof(status));
                reply_length += sizeof(status);
            }
            break;
        }

        case GS_FIN_CMD_SET_CUSTOM_POSITION: {
            gs_fin_positions_t pos;

            /* Copy received positions (angles) to struct */
            memcpy(&pos, &packet->data[1], sizeof(pos));

            /* Ensure correct endianness */
            uint16_t * pos_array = (uint16_t *) &pos; //(void *)
            for (int i = 0; i < 4; i++) {
                pos_array[i] = csp_ntoh16(pos_array[i]);
            }
            /* Pass positions to internal layer */
            int8_t error = set_fin_pos(&pos);

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        case GS_FIN_CMD_SET_MAX_DRAG: {
            /* Pass command to internal layer */
            int8_t error = set_max_drag();

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        case GS_FIN_CMD_SET_MIN_DRAG: {
            /* Pass command to internal layer */
            int8_t error = set_min_drag();

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }
		
        case UNIMAN_FIN_CFG_GET: {
	       //get current config and send it
		   
		   
	        break;
        }
		
        case UNIMAN_FIN_CFG_SET: {
	        //get set current config based on incoming
	        
	        
	        break;
        }		

        default: {
            /* Set error code in response to no command found */
            packet->data[0] = FIN_CMD_NOT_FOUND;
            break;
        }
    }

    /* Send response */
    packet->length = reply_length;
    if (!csp_send(conn, packet, 0)) {
        csp_buffer_free(packet);
    }
}

/**
   CSP server task.

   Handles generic CSP cmds and fin controller cmds.
   CSP must be enabled prior to creation of this task.
 */


CSP_DEFINE_TASK(task_server)
{
    /* Create socket without any socket options */
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);

    /* Bind all ports to socket */
    csp_bind(sock, CSP_ANY);

    /* Create 10 connections backlog queue */
    csp_listen(sock, 10);

    /* Pointer to current connection and packet */
    csp_conn_t * conn;
    csp_packet_t * packet;

    /* Process incoming connections */
    while (1) {
csp_log_info("%s %d\n",pcTaskGetName(NULL),uxTaskGetStackHighWaterMark2(NULL));
        /* Wait for connection, 10000 ms timeout */
        if ((conn = csp_accept(sock, 10000)) == NULL) {
            continue;
        }

        /* Read packets */
        while ((packet = csp_read(conn, 0)) != NULL) { //was 100 in my code
            switch (csp_conn_dport(conn)) {
                case GS_FIN_CONTROL_PORT:
                    /* Process fin controller packet here */
                    process_fin_cmd(conn, packet);
                    break;

                default:
                    /* Let the service handler reply pings, buffer use, etc. */
                    csp_service_handler(conn, packet);
                    break;
            }
        }

        /* Close current connection, and handle next */
        csp_close(conn);

    }

    return CSP_TASK_RETURN;
}





