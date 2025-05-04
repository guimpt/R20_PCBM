/*
 * mct8316.h
 *
 *  Created on: Mar 16, 2025
 *      Author: Guim
 */

#ifndef INC_MCT8316_H_
#define INC_MCT8316_H_

/* Includes */
#include <stdint.h>
#include "stm32f0xx_hal.h"

/* Defines */

#define MCT8316_MSG_LENGTH			16
#define MCT8316_MSG_RW_BIT			15
#define MCT8316_MSG_ADDRESS_BIT		9
#define MCT8316_MSG_PARITY_BIT		8
#define MCT8316_MSG_W				0
#define MCT8316_MSG_R				1
#define MCT8316_SPI_TIMEOUT			100

// Registers
#define MCT8316_REG_IC_STATUS		0x00
#define MCT8316_REG_STATUS_1		0x01
#define MCT8316_REG_STATUS_2		0x02
#define MCT8316_REG_CTRL_1			0x03
#define MCT8316_REG_CTRL_2			0x04
#define MCT8316_REG_CTRL_3			0x05
#define MCT8316_REG_CTRL_4			0x06
#define MCT8316_REG_CTRL_5			0x07
#define MCT8316_REG_CTRL_6			0x08
#define MCT8316_REG_CTRL_7			0x09
#define MCT8316_REG_CTRL_8			0x0A
#define MCT8316_REG_CTRL_9			0x0B
#define MCT8316_REG_CTRL_10			0x0C

// MCT8316 IC Status Register (0x00)
#define MCT8316_IC_STATUS_MTR_LOCK  (1 << 7) /* 1 if motor lock detected */
#define MCT8316_IC_STATUS_BK_FLT    (1 << 6) /* 1 if buck regulator fault detected */
#define MCT8316_IC_STATUS_SPI_FLT   (1 << 5) /* 1 if SPI fault detected */
#define MCT8316_IC_STATUS_OCP       (1 << 4) /* 1 if overcurrent detected */
#define MCT8316_IC_STATUS_NPOR      (1 << 3) /* 1 if power-on reset detected */
#define MCT8316_IC_STATUS_OVP       (1 << 2) /* 1 if overvoltage detected */
#define MCT8316_IC_STATUS_OT        (1 << 1) /* 1 if overtemperature detected */
#define MCT8316_IC_STATUS_FAULT     (1 << 0) /* 1 if fault condition detected */

// MCT8316 Status Register 1 (0x01)
#define MCT8316_STATUS_1_OTW        (1 << 7) /* 1 if overtemperature warning */
#define MCT8316_STATUS_1_OTS        (1 << 6) /* 1 if overtemperature shutdown detected */
#define MCT8316_STATUS_1_OCP_HC     (1 << 5) /* 1 if overcurrent on high-side switch OUTC */
#define MCT8316_STATUS_1_OCL_LC     (1 << 4) /* 1 if overcurrent on low-side switch OUTC */
#define MCT8316_STATUS_1_OCP_HB     (1 << 3) /* 1 if overcurrent on high-side switch OUTB */
#define MCT8316_STATUS_1_OCP_LB     (1 << 2) /* 1 if overcurrent on low-side switch OUTB */
#define MCT8316_STATUS_1_OCP_HA     (1 << 1) /* 1 if overcurrent on high-side switch OUTA */
#define MCT8316_STATUS_1_OCP_LA     (1 << 0) /* 1 if overcurrent on low-side switch OUTA */

// MCT8316 Status Register 2 (0x02)
#define MCT8316_STATUS_2_OTP_ERR      (1 << 6) /* 1 if OTP error detected */
#define MCT8316_STATUS_2_BUCK_OCP     (1 << 5) /* 1 if buck regulator overcurrent detected */
#define MCT8316_STATUS_2_BUCK_UV      (1 << 4) /* 1 if buck regulator undervoltage detected */
#define MCT8316_STATUS_2_VCP_UV       (1 << 3) /* 1 if charge pump undervoltage detected */
#define MCT8316_STATUS_2_SPI_PARITY   (1 << 2) /* 1 if SPI parity error detected */
#define MCT8316_STATUS_2_SPI_SCLK_FLT (1 << 1) /* 1 if SPI clock framing error detected */
#define MCT8316_STATUS_2_SPI_ADDR_FLT (1 << 0) /* 1 if SPI address fault detected */

// MCT8316 Control Register 1 (0x03)
#define MCT8316_CTRL_1_REG_LOCK_MASK    (0x07 << 0) /* Register lock bits */
#define MCT8316_CTRL_1_REG_UNLOCK       (0x03 << 0) /* Unlock all registers */
#define MCT8316_CTRL_1_REG_LOCK         (0x06 << 0) /* Lock all registers */

// MCT8316 Control Register 2 (0x04)
#define MCT8316_CTRL_2_SDO_MODE         (1 << 5) /* 1 for SDO push-pull, 0 for open-drain */
#define MCT8316_CTRL_2_SLEW_MASK        (0x03 << 3) /* Slew rate selection */
#define MCT8316_CTRL_2_PWM_MODE_MASK    (0x03 << 1) /* PWM mode selection */
#define MCT8316_CTRL_2_CLR_FLT          (1 << 0) /* 1 to clear fault flags */

// MCT8316 Control Register 3 (0x05)
#define MCT8316_CTRL_3_PWM_100_DUTY_SEL (1 << 4) /* 1 for 40kHz PWM, 0 for 20kHz */
#define MCT8316_CTRL_3_OVP_SEL          (1 << 3) /* 1 for 22V overvoltage, 0 for 34V */
#define MCT8316_CTRL_3_OVP_EN           (1 << 2) /* 1 to enable overvoltage protection */
#define MCT8316_CTRL_3_OTW_REP          (1 << 0) /* 1 to enable overtemperature warning reporting */

// MCT8316 Control Register 4 (0x06)
#define MCT8316_CTRL_4_DRV_OFF          (1 << 7) /* 1 to enter low-power standby mode */
#define MCT8316_CTRL_4_OCP_CBC          (1 << 6) /* 1 to enable OCP cycle clearing */
#define MCT8316_CTRL_4_OCP_DEG_MASK     (0x03 << 4) /* OCP deglitch time selection */
#define MCT8316_CTRL_4_OCP_RETRY        (1 << 3) /* 1 for 500ms OCP retry, 0 for 5ms */
#define MCT8316_CTRL_4_OCP_LVL          (1 << 2) /* 1 for 24A OCP level, 0 for 16A */
#define MCT8316_CTRL_4_OCP_MODE_MASK    (0x03 << 0) /* Overcurrent mode selection */

// MCT8316 Control Register 5 (0x07)
#define MCT8316_CTRL_5_ILIM_RECIR       (1 << 6) /* 1 for diode recirculation, 0 for FET recirculation */
#define MCT8316_CTRL_5_EN_AAR           (1 << 3) /* 1 to enable AAR mode */
#define MCT8316_CTRL_5_EN_ASR           (1 << 2) /* 1 to enable ASR mode */
#define MCT8316_CTRL_5_CSA_GAIN_MASK    (0x03 << 0) /* Current sense amplifier gain selection */

// MCT8316 Control Register 6 (0x08)
#define MCT8316_CTRL_6_BUCK_PS_DIS      (1 << 4) /* 1 to disable buck power sequencing */
#define MCT8316_CTRL_6_BUCK_CL          (1 << 3) /* 1 for 150mA buck current limit, 0 for 600mA */
#define MCT8316_CTRL_6_BUCK_SEL_MASK    (0x03 << 1) /* Buck voltage selection */
#define MCT8316_CTRL_6_BUCK_DIS         (1 << 0) /* 1 to disable buck regulator */

// MCT8316 Control Register 7 (0x09)
#define MCT8316_CTRL_7_HALL_HYS         (1 << 4) /* 1 for 50mV Hall hysteresis, 0 for 5mV */
#define MCT8316_CTRL_7_BRAKE_MODE       (1 << 3) /* 1 for coast mode, 0 for brake mode */
#define MCT8316_CTRL_7_COAST            (1 << 2) /* 1 to enable coast mode */
#define MCT8316_CTRL_7_BRAKE            (1 << 1) /* 1 to enable brake mode */
#define MCT8316_CTRL_7_DIR              (1 << 0) /* 1 for counter-clockwise, 0 for clockwise */

// MCT8316 Control Register 8 (0x0A)
#define MCT8316_CTRL_8_FGOUT_SEL_MASK   (0x03 << 6) /* FGOUT frequency selection */
#define MCT8316_CTRL_8_MTR_LOCK_RETRY   (1 << 4) /* 1 for 5000ms retry, 0 for 500ms */
#define MCT8316_CTRL_8_MTR_LOCK_TDET_MASK (0x03 << 2) /* Motor lock detection time selection */
#define MCT8316_CTRL_8_MTR_LOCK_MODE_MASK (0x03 << 0) /* Motor lock fault behavior selection */

// MCT8316 Control Register 9 (0x0B)
#define MCT8316_CTRL_9_ADVANCE_LVL_MASK (0x07 << 0) /* Phase advance selection */

// MCT8316 Control Register 10 (0x0C)
#define MCT8316_CTRL_10_DLYCMP_EN       (1 << 4) /* 1 to enable delay compensation */
#define MCT8316_CTRL_10_DLY_TARGET_MASK (0x0F << 0) /* Delay target selection */



/* Typedefs */
typedef struct
{
	SPI_HandleTypeDef *hspi;
	uint8_t ic_status;
	uint8_t status_1;
	uint8_t status_2;
} MCT8316;


/* Public variables definitions */

/* Public functions definitions */
HAL_StatusTypeDef MCT8316_Init(MCT8316 *mct8316);
HAL_StatusTypeDef MCT8316_UpdateStatus(MCT8316 *mct8316);
HAL_StatusTypeDef MCT8316_SetDirection(MCT8316 *mct8316, uint8_t direction);
HAL_StatusTypeDef MCT8316_Brake(MCT8316 *mct8316, uint8_t enable);
HAL_StatusTypeDef MCT8316_ClearFaults(MCT8316 *mct8316);

#endif /* INC_MCT8316_H_ */
