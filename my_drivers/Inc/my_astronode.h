/*
 * my_astronode.h
 *
 *  Created on: Oct 23, 2023
 *      Author: mzeml
 */

#ifndef ASTROCAST_INC_MY_ASTRONODE_H_
#define ASTROCAST_INC_MY_ASTRONODE_H_

#include "astronode_definitions.h"
#include "astronode_application.h"


void reset_astronode ( void ) ;
void send_astronode_request ( uint8_t* , uint32_t ) ;
void send_debug_logs ( char* ) ;
void send_astronode_request ( uint8_t* , uint32_t ) ;
bool is_astronode_character_received ( uint8_t* ) ;
bool is_systick_timeout_over ( uint32_t , uint16_t ) ;
uint32_t get_systick ( void ) ;

#endif /* ASTROCAST_INC_MY_ASTRONODE_H_ */
