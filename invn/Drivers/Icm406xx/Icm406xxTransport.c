/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
 
#include "Icm406xxTransport.h"
#include "Icm406xxDefs.h"

#include "../../InvError.h"


/* Function definition */
static uint8_t * get_register_cache_addr(struct inv_icm406xx * s, uint8_t reg);
static uint8_t is_aux_interface(struct inv_icm406xx_transport *t);


int inv_icm406xx_init_transport(struct inv_icm406xx * s)
{
	// Registers in cache must be in bank 0
	int status = 0;
	struct inv_icm406xx_transport *t = (struct inv_icm406xx_transport *)s;

	if(!is_aux_interface(t)){
		status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG1,  &(t->register_cache.intf_cfg_1_reg), 1);
		status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0,    &(t->register_cache.pwr_mngt_0_reg), 1);
		status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG0,  &(t->register_cache.gyro_cfg_0_reg), 1);
		status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG0, &(t->register_cache.accel_cfg_0_reg), 1);
		status |= t->serif.read_reg(&(t->serif), MPUREG_TMST_CONFIG,   &(t->register_cache.tmst_cfg_reg), 1);
	}
	
	status |= t->serif.read_reg(&(t->serif), MPUREG_REG_BANK_SEL,  &(t->register_cache.bank_sel_reg), 1);

	return status;
}

int inv_icm406xx_read_reg(struct inv_icm406xx * s, uint8_t reg, uint32_t len, uint8_t * buf)
{

	// First field of struct inv_icm406xx is assumed to be a struct inv_icm406xx_transport object.
	// So let's cast s to struct inv_icm406xx_transport and ignore the rest of struct inv_icm406xx.
	struct inv_icm406xx_transport *t = (struct inv_icm406xx_transport *)s;
	//uint32_t i=0;
	// Physical access to read registers
	//if((len-i) > t->serif.max_read)
	   if((len) > t->serif.max_read)
		return INV_ERROR_SIZE;
	//if(t->serif.read_reg(&(t->serif), reg+i, &buf[i], len-i) != 0)
	if(t->serif.read_reg(&(t->serif), reg, &buf[0], len) != 0)
		return INV_ERROR_TRANSPORT;
	
	return 0;
}

int inv_icm406xx_write_reg(struct inv_icm406xx * s, uint8_t reg, uint32_t len, const uint8_t * buf)
{


	// First field of struct inv_icm406xx is assumed to be a struct inv_icm406xx_transport object.
	// So let's cast s to struct inv_icm406xx_transport and ignore the rest of struct inv_icm406xx.
	struct inv_icm406xx_transport *t = (struct inv_icm406xx_transport *)s;
	uint32_t i;
	
	if(len > t->serif.max_write)
		return INV_ERROR_SIZE;
	
	// Physical access to write registers
	if(t->serif.write_reg(&(t->serif), reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;
	
	return 0;
}

/* Static function */

/* MPUREG_REG_BANK_SEL shall never be added to the function get_register_cache_addr() */
static uint8_t * get_register_cache_addr(struct inv_icm406xx * s, uint8_t reg)
{
	struct inv_icm406xx_transport *t = (struct inv_icm406xx_transport *)s;

	switch(reg) {
		case MPUREG_INTF_CONFIG1:     return &(t->register_cache.intf_cfg_1_reg);
		case MPUREG_PWR_MGMT_0:       return &(t->register_cache.pwr_mngt_0_reg);
		case MPUREG_GYRO_CONFIG0:     return &(t->register_cache.gyro_cfg_0_reg);
		case MPUREG_ACCEL_CONFIG0:    return &(t->register_cache.accel_cfg_0_reg);
		case MPUREG_TMST_CONFIG:      return &(t->register_cache.tmst_cfg_reg);
		default:                      return (uint8_t *)0; // Not found
	}
}

static uint8_t is_aux_interface(struct inv_icm406xx_transport *t)
{
	if ((t->serif.serif_type == ICM406XX_AUX1_SPI3) || (t->serif.serif_type == ICM406XX_AUX2_SPI3))
		return 1;
	else
		return 0;
}
