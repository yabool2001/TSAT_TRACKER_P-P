/*
 * my_lis2dw12.c
 *
 *  Created on: Nov 9, 2023
 *      Author: mzeml
 */

#include "my_lis2dw12.h"

bool my_lis2dw12_init ( stmdev_ctx_t* ctx )
{
	uint8_t rst = 1 ;

	/*Restore default configuration */
	lis2dw12_reset_set ( ctx , PROPERTY_ENABLE ) ;
	do {
		lis2dw12_reset_get ( ctx, &rst ) ;
	} while ( rst ) ;
	if ( my_lis2dw12_get_id ( ctx ) == LIS2DW12_ID )
	{
		lis2dw12_full_scale_set 	( ctx , LIS2DW12_2g ) ;
		lis2dw12_power_mode_set 	( ctx , LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit ) ;
		lis2dw12_data_rate_set 		( ctx , LIS2DW12_XL_ODR_200Hz ) ;
		lis2dw12_filter_path_set 	( ctx , LIS2DW12_HIGH_PASS_ON_OUT ) ;
		lis2dw12_wkup_dur_set		( ctx , 0 ) ;
		lis2dw12_wkup_threshold_set	( ctx, 2 ) ;
		return true ;
	}
	return 0 ;
}

uint8_t my_lis2dw12_get_id ( stmdev_ctx_t* ctx )
{
	uint8_t id = 0 ;
	lis2dw12_device_id_get ( ctx , &id ) ;
	return id ;
}

void my_lis2dw12_int1_wu_enable ( stmdev_ctx_t* ctx )
{
	lis2dw12_reg_t int_route ;
	lis2dw12_pin_int1_route_get	( ctx, &int_route.ctrl4_int1_pad_ctrl ) ;
	int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE ;
	lis2dw12_pin_int1_route_set	( ctx , &int_route.ctrl4_int1_pad_ctrl ) ;
}
void my_lis2dw12_int1_wu_disable ( stmdev_ctx_t* ctx )
{
	lis2dw12_reg_t int_route ;
	lis2dw12_pin_int1_route_get	( ctx, &int_route.ctrl4_int1_pad_ctrl ) ;
	int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_DISABLE ;
	lis2dw12_pin_int1_route_set	( ctx , &int_route.ctrl4_int1_pad_ctrl ) ;
}

/*
void my_is2dw12_print_conf ( void* h )
{
	uint8_t reg[20] ;
	char dbg_buff[25] ;

	lis2dw12_wkup_threshold_get ( h , reg ) ;
	sprintf ( dbg_buff , "WAKE_UP_THS: %h\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_CTRL1 , reg , 1 ) ;
	sprintf ( dbg_buff , "CTRL1: %x\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_CTRL3 , reg , 1 ) ;
	sprintf ( dbg_buff , "CTRL3: %d\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_CTRL4_INT1_PAD_CTRL , reg , 1 ) ;
	sprintf ( dbg_buff , "CTRL4: %d\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_CTRL5_INT2_PAD_CTRL , reg , 1 ) ;
	sprintf ( dbg_buff , "CTRL5: %d\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_CTRL6 , reg , 1 ) ;
	sprintf ( dbg_buff , "CTRL6: %d\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_STATUS , reg , 1 ) ;
	sprintf ( dbg_buff , "STATUS: %d\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;

	lis2dw12_read_reg ( h , LIS2DW12_WAKE_UP_SRC , reg , 1 ) ;
	sprintf ( dbg_buff , "WAKE_UP_SRC: %d\r\n" , reg ) ;
	send_debug_logs ( dbg_buff ) ;
}
*/
