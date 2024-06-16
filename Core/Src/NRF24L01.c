\/UIK<"O:| 
5/*
 * NRF24L01.c
 *
 *  Created on: Apr 14, 2024
 *      Author: ADMIN
 */
#include "stm32f1xx_hal.h"
#include "NRF24L01.h"


extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT GPIOB
#define NRF24_CE_PIN GPIO_PIN_0

#define NRF24_CSN_PORT GPIOA
#define NRF24_CSN_PIN GPIO_PIN_4

// a Function to select or not some slave devices
void CS_Select (void) {
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}
void CS_UnSelect (void) {
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

// a Function to enable or not some slave devices
void CE_Enable (void) {
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}
void CE_Disable (void) {
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}


// RESET FUNC


// WRITE
// write a single byte to the particular Reg
void nrf24_WriteReg (uint8_t Reg, uint8_t Data) {
	uint8_t buf[2];
	buf[0] = 1 << 5 | Reg; // see data sheet page 48
	buf[1] = Data;

	// pull the CS pin LOW to select the device data sheet page 47
	CS_Select();

	// Define above could have some error
	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	// Pull the CS HIGH to release the device

	CS_UnSelect();
}

// write multiple bytes starting from particular Reg
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size) {
		uint8_t buf[1];
		buf[0] = 1 << 5 | Reg; // see data sheet page 48

		// pull the CS pin LOW to select the device data sheet page 47
		CS_Select();

		// Define above could have some error
		HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
		HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);
		// Pull the CS HIGH to release the device

		CS_UnSelect();
}


// READ BYTE
uint8_t nrf24_ReadReg (uint8_t Reg) {
	uint8_t data = 0;
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 1000);

	CS_UnSelect();

	return data;
}

void nrf24_ReadRegMulti (uint8_t Reg, uint8_t *data, int size) {
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	CS_UnSelect();
}


// SEND CMD

void nrf24_SendCmd (uint8_t cmd) {
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	CS_UnSelect();

}

// RESET FUNC

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}



void nrf24_Init (void) {
	// disable the chip before config device
	CE_Disable();


	nrf24_reset(0);
	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	//config later
	nrf24_WriteReg(CONFIG, 0);

	// disable auto auto acknowledgement
	nrf24_WriteReg(EN_AA, 0);

	// data pipes config later
	nrf24_WriteReg(EN_RXADDR, 0);

	// setup address width
	nrf24_WriteReg(SETUP_AW, 0x03);

	// since were not using AA so no retransmit
	nrf24_WriteReg(SETUP_RETR,0);

	// channel selection for the transmitter (setup later)
	nrf24_WriteReg(RF_CH, 0);

	// setting bandwidth and power signal to maximum
	nrf24_WriteReg(RF_SETUP, 0x0E);

	uint8_t config1 = nrf24_ReadReg(0x00);
	CE_Enable();

}

// Setup Tx MODE

void nrf24_TxMode (uint8_t *Address, uint8_t channel) {


	// disable the chip before config device
	CE_Disable();
	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);
	// select the channel
	nrf24_WriteReg(RF_CH, channel);

	// config the address of transmit
	nrf24_WriteRegMulti(TX_ADDR, Address, 5);

	// power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1);
	nrf24_WriteReg(CONFIG, config);

	// enable the chip after config the device
	CE_Enable();
}

// transmit the data

uint8_t nrf24_Transmit (uint8_t *data) {

	uint8_t cmd = 0;

	// select the device
	CS_Select();

	// send payload command see data sheet
	cmd = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// send the payload itself
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	//release the device
	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	// check if the data is done transmited
	if ( (fifostatus&(1<<4)) && (!(fifostatus&(1<<3)))) // checks if the 4th bit is set and 3th bit is reset
	{
		cmd = FLUSH_TX;
		nrf24_SendCmd(cmd);

		nrf24_reset(FIFO_STATUS);
		return 1;
	}

	return 0;
}


void nrf25_RxMode (uint8_t *Address, uint8_t channel) {
	// disable the chip before config device
		CE_Disable();

		// select the channel
		nrf24_WriteReg(RF_CH, channel);


		// select data pipe 1
		uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
		en_rxaddr = en_rxaddr | (1<<1);
		nrf24_WriteReg(EN_RXADDR, en_rxaddr);

		// config the address of transmit
		nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);

		// setup data width to 32 bytes for pipe 1

		nrf24_WriteReg(RX_PW_P1,32);

		// power up the device in Rx Mode
		uint8_t config = nrf24_ReadReg(CONFIG);
		config = config | (1<<1) | (1<<0);
		nrf24_WriteReg(CONFIG, config);

		// enable the device after config it
		CE_Enable();
}


// i dont rly understand this function

uint8_t isDataAvailable (int pipenum) {
	uint8_t status = nrf24_ReadReg(STATUS);

	if (  (status&(1<<6)) && (status&(pipenum<<1)) ) {
		nrf24_WriteReg(STATUS, (1<<6));
		return 1;
	}
	return 0;
}

void nrf24_Receive (uint8_t *data) {

	uint8_t cmd = 0;

		// select the device
		CS_Select();

		// send receive command see data sheet
		cmd = R_RX_PAYLOAD;
		HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

		// receive the payload itself
		HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

		//release the device
		CS_UnSelect();

		HAL_Delay(1);

		// the data is received, flush the RX fifo

	cmd = FLUSH_RX;
	nrf24_SendCmd(cmd);

}


