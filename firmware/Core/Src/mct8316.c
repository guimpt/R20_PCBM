/*
 * mct8316.c
 *
 *  Created on: Mar 16, 2025
 *      Author: Guim
 */


#include "mct8316.h"

/* Private function definitions */
static HAL_StatusTypeDef MCT8316_Write(MCT8316 *mct8316, uint8_t address, uint8_t data);
static HAL_StatusTypeDef MCT8316_Read(MCT8316 *mct8316, uint8_t address, uint8_t *data);

/* Private variables definitions */

/* Public functions */
HAL_StatusTypeDef MCT8316_Init(MCT8316 *mct8316) {
    HAL_StatusTypeDef status;
    uint8_t data_r;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure PA4 as output push-pull (manual CS control)
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;      // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // No pull-up/pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // High speed for SPI
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize CS to HIGH (inactive)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);

    // Unlock all registers
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_1, MCT8316_CTRL_1_REG_UNLOCK);
    if (status != HAL_OK) return status;
    status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_1, &data_r);

    // Set SDO to push-pull mode
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_2, MCT8316_CTRL_2_SDO_MODE | (1 << 1));  // PWM_MODE = 3 (Digital Hall Input Synchronous ASR/AAR)
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_2, &data_r);

    // Enable 22V overvoltage protection, 40kHz PWM
//    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_3, MCT8316_CTRL_3_PWM_100_DUTY_SEL);
//    if (status != HAL_OK) return status;
//	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_3, &data_r);

    // Configure overcurrent protection (latched at 16A)
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_4, (0 << 2) |  // OCP_LVL = 0 (16A)
                                                       (0 << 0));  // OCP_MODE = 0 (latched)
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_4, &data_r);

    // Enable ILIM FET recirculation, max CSA gain
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_5, MCT8316_CTRL_5_ILIM_RECIR |
                                                       (3 << 0));  // CSA_GAIN = 3 (1.2 V/A)
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_5, &data_r);

    // Disable buck regulator
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_6, MCT8316_CTRL_6_BUCK_DIS);
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_6, &data_r);

    // Motor lock warning only
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_8, (2 << 0));  // MTR_LOCK_MODE = 2 (report only)
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_8, &data_r);

    // No phase advance
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_9, 0);
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_9, &data_r);

    // No delay compensation
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_10, 0);
    if (status != HAL_OK) return status;
	status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_10, &data_r);

    // Verify initialization by reading back a register (e.g., IC Status)
    uint8_t ic_status;
	status = MCT8316_Read(mct8316, MCT8316_REG_IC_STATUS, &ic_status);
    if (status != HAL_OK) return status;
    if (ic_status == 0xFF) return HAL_ERROR; // Assuming 0xFF means SPI failure TODO improve this check

    return HAL_OK;
}

// Update status registers by reading IC status, Status 1, and Status 2
HAL_StatusTypeDef MCT8316_UpdateStatus(MCT8316 *mct8316) {
    HAL_StatusTypeDef status;

//    status = MCT8316_Read(mct8316, MCT8316_REG_IC_STATUS, &mct8316->ic_status);
//    if (status != HAL_OK) return status;

    status = MCT8316_Read(mct8316, MCT8316_REG_STATUS_1, &mct8316->status_1);
    if (status != HAL_OK) return status;

    status = MCT8316_Read(mct8316, MCT8316_REG_STATUS_2, &mct8316->status_2);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

// Change motor direction
HAL_StatusTypeDef MCT8316_SetDirection(MCT8316 *mct8316, uint8_t direction) {
    if (direction > 1) return HAL_ERROR; // Direction must be 0 (CW) or 1 (CCW)

    uint8_t ctrl7;
    HAL_StatusTypeDef status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_7, &ctrl7);
    if (status != HAL_OK) return status;

    ctrl7 = (ctrl7 & ~MCT8316_CTRL_7_DIR) | (direction << 0);
    status = MCT8316_Write(mct8316, MCT8316_REG_CTRL_7, ctrl7);
    if (status != HAL_OK) return status;

    status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_7, &ctrl7);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

// Apply brake
HAL_StatusTypeDef MCT8316_Brake(MCT8316 *mct8316, uint8_t enable) {
    if (enable > 1) return HAL_ERROR; // Brake must be 0 (disable) or 1 (enable)

    uint8_t ctrl7;
    HAL_StatusTypeDef status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_7, &ctrl7);
    if (status != HAL_OK) return status;

    ctrl7 = (ctrl7 & ~MCT8316_CTRL_7_BRAKE) | (enable << 1);
    return MCT8316_Write(mct8316, MCT8316_REG_CTRL_7, ctrl7);
}

// Reset fault flags
HAL_StatusTypeDef MCT8316_ClearFaults(MCT8316 *mct8316) {
    uint8_t ctrl2;
    HAL_StatusTypeDef status = MCT8316_Read(mct8316, MCT8316_REG_CTRL_2, &ctrl2);
    if (status != HAL_OK) return status;

    ctrl2 |= MCT8316_CTRL_2_CLR_FLT;  // Set the clear fault bit (W1C)

    return MCT8316_Write(mct8316, MCT8316_REG_CTRL_2, ctrl2);
}


/* Private functions */
static HAL_StatusTypeDef MCT8316_Write(MCT8316 *mct8316, uint8_t address, uint8_t data){
	HAL_StatusTypeDef status;
	uint16_t tx_message = (uint16_t) data, rx_message, purge;
	tx_message |= (MCT8316_MSG_W << MCT8316_MSG_RW_BIT) | (address << MCT8316_MSG_ADDRESS_BIT);
	/* Compute parity */
	uint8_t parity = 0;
	uint16_t n = tx_message;
	while (n)
	{
		parity = !parity;
		n = n & (n - 1);
	}

	if (parity) tx_message |= (1 << MCT8316_MSG_PARITY_BIT);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(mct8316->hspi, (uint8_t *) &tx_message, (uint8_t *) &rx_message, 1, MCT8316_SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	/* Purge SPI bus */
//	HAL_Delay(1);
//	tx_message = 0;
//
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	status = HAL_SPI_TransmitReceive(mct8316->hspi, (uint8_t *) &tx_message, (uint8_t *) &purge, 1, MCT8316_SPI_TIMEOUT);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return status;
}

static HAL_StatusTypeDef MCT8316_Read(MCT8316 *mct8316, uint8_t address, uint8_t *data){
	HAL_StatusTypeDef status;
	uint16_t tx_message = 0, rx_message, purge;
	tx_message |= (address << MCT8316_MSG_ADDRESS_BIT);
	tx_message |= (MCT8316_MSG_R << MCT8316_MSG_RW_BIT);
	/* Compute parity */
	uint8_t parity = 0;
	uint16_t n = tx_message;
	while (n)
	{
		parity = !parity;
		n = n & (n - 1);
	}

	if (parity) tx_message |= (1 << MCT8316_MSG_PARITY_BIT);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(mct8316->hspi, (uint8_t *) &tx_message, (uint8_t *) &rx_message, 1, MCT8316_SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	if (status != HAL_OK) return status;

	/* Purge SPI bus */
//	HAL_Delay(1);
//	tx_message = 0;
//
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	status = HAL_SPI_TransmitReceive(mct8316->hspi, (uint8_t *) &tx_message, (uint8_t *) &purge, 1, MCT8316_SPI_TIMEOUT);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    *data = (uint8_t) (rx_message & 0xFF);
    mct8316->ic_status = (uint8_t) (rx_message >> 8);

	return HAL_OK;
}
