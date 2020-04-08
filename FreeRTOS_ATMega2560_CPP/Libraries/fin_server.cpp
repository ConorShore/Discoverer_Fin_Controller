/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */
/**
   @file
   Fin controller server (implemented on Fin Controller Board)
*/

#include <stdio.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <csp/arch/csp_thread.h>
#include <fin.h>
#include <fin_server.h>
#include <fin_server_backend.h>

//TODO - remove this library, only for testing
#include <R_EEPROM.h>

/**
   Processes incoming fin controller command.
 */
static void process_fin_cmd(csp_conn_t * conn, csp_packet_t * packet)
{
    uint8_t reply_length = 1;
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

				
				//R_EEPROM testg;
				
                memcpy(&packet->data[1], &status, sizeof(status));

	
				//testg.write(&status,sizeof(status));
				
				
				for (uint16_t i=1; i<sizeof(uint16_t)*16+1;i+=2){
					//correct endianness
					uint8_t temp =packet->data[i+1];
					packet->data[i+1]=packet->data[i];
					packet->data[i]=temp;
				}
	
		
				
				packet->data[33]=status.mode;
// 				packet->data[34]=0;
// 				packet->data[35]=0;
// 				packet->data[36]=0;
// 				packet->data[37]=status.status_code;
				packet->data[34]=status.status_code;
				
				
			
// 				for(int i=0 ;i<38;i++) {
// 					printf("%x ",packet->data[i]);
// 				}
// 				
                reply_length += sizeof(status)+3;
				//csp_log_info("sizeof sat %d\n",reply_length);
            }
            break;
        }

        case GS_FIN_CMD_SET_CUSTOM_POSITION: {
            gs_fin_positions_t pos;

            /* Copy received positions (angles) to struct */
            memcpy(&pos, &packet->data[1], sizeof(pos));

            /* Ensure correct endianness */
            uint16_t * pos_array = (uint16_t*) &pos;
            for (int i = 0; i < 4; i++) {
                pos_array[i] = csp_ntoh16(pos_array[i]);
            }
            /* Pass positions to internal layer */
            int8_t error = set_fin_pos(&pos);

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        case GS_FIN_CMD_CFG_GET: {
            gs_fin_config_t conf;

            /* Get conf internally */
            int8_t error = get_fin_config(&conf);

            /* Set error code in response */
            packet->data[0] = error;

            if (error == FIN_CMD_OK) {
                /* Copy conf to response buffer */
                memcpy(&packet->data[1], &conf, sizeof(conf));
                reply_length += sizeof(conf);
            }
            break;
        }

        case GS_FIN_CMD_CFG_SET: {
            gs_fin_config_t conf;

            /* Copy received conf to struct */
            memcpy(&conf, &packet->data[1], sizeof(conf));

            /* Ensure correct endianness */
            conf.stepper_speed = csp_ntoh16(conf.stepper_speed);

            /* Pass conf to internal layer */
            int8_t error = set_fin_config(&conf);

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        case GS_FIN_CMD_CFG_SAVE: {
            /* Pass conf to internal layer */
            int8_t error = save_fin_config();

            /* Set error code in response */
            packet->data[0] = error;
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

        /* Wait for connection, 10000 ms timeout */
        if ((conn = csp_accept(sock, 10000)) == NULL) {
            continue;
        }

        /* Read packets */
        while ((packet = csp_read(conn, 0)) != NULL) {
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