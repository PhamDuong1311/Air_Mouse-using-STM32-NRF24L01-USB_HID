#include "stm32f1xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define CE_PORT GPIOB    // Chip enable activate Rx or Tx mode
#define CE_PIN GPIO_PIN_0

#define CSN_PORT GPIOA // Chip select
#define CSN_PIN GPIO_PIN_4

void CS_Select(){
	HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, 0);
}

void CS_UnSelect(){
	HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, 1);
}

void CE_Enable(){
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, 1);
}

void CE_Disable(){
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, 0);
}

// Write single byte to the particular register
void nrf24_Write_Register(uint8_t Register, uint8_t Data){

	uint8_t Buff[2];
	Buff[0] = Register|1<<5;
	Buff[1] = Data;

	// Pull the CS PIN LOW to select the device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, Buff, 2, 1000);
	// Pull the CS PIN HIGH to release the device
	CS_UnSelect();

}


// Write multiple byte to the particular register
void nrf24_Write_Register_Multiple(uint8_t Register, uint8_t *Data, uint8_t size){
	uint8_t Buff[2];
	Buff[0] = Register|1<<5;
//	Buff[1] = Data;

	// Pull the CS PIN LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, Buff, 1, 1000);
	HAL_SPI_Transmit(NRF24_SPI, Data, size, 1000);
	// Pull the CS PIN HIGH to release the device
	CS_UnSelect();
}

// Read single byte from particular register
uint8_t nrf24_Read_Register(uint8_t Register){

	uint8_t data = 0;
	// Pull the CS PIN LOW to select the device
	CS_Select();
	// Send address
	HAL_SPI_Transmit(NRF24_SPI, &Register, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);
	// Pull the CS PIN HIGH to release the device
	CS_UnSelect();

	return data;
}

// Read multiple bytes from particular register

void nrf24_Read_Register_Multiple(uint8_t Register, uint8_t *Data, int size){

	// Pull the CS PIN LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Register, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, Data, size, 1000);
	// Pull the CS PIN HIGH to release the device
	CS_UnSelect();
}

// Send command to NRF24L01
void nrf24_Send_Cmd(uint8_t cmd){

	// Pull the CS PIN LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// Pull the CS PIN HIGH to release the device
	CS_UnSelect();
}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_Write_Register(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_Write_Register(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_Write_Register(CONFIG, 0x08);
	nrf24_Write_Register(EN_AA, 0x3F);
	nrf24_Write_Register(EN_RXADDR, 0x03);
	nrf24_Write_Register(SETUP_AW, 0x03);
	nrf24_Write_Register(SETUP_RETR, 0x03);
	nrf24_Write_Register(RF_CH, 0x02);
	nrf24_Write_Register(RF_SETUP, 0x0E);
	nrf24_Write_Register(STATUS, 0x00);
	nrf24_Write_Register(OBSERVE_TX, 0x00);
	nrf24_Write_Register(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_Read_Register_Multiple(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_Read_Register_Multiple(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_Write_Register(RX_ADDR_P2, 0xC3);
	nrf24_Write_Register(RX_ADDR_P3, 0xC4);
	nrf24_Write_Register(RX_ADDR_P4, 0xC5);
	nrf24_Write_Register(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_Read_Register_Multiple(TX_ADDR, tx_addr_def, 5);
	nrf24_Write_Register(RX_PW_P0, 0);
	nrf24_Write_Register(RX_PW_P1, 0);
	nrf24_Write_Register(RX_PW_P2, 0);
	nrf24_Write_Register(RX_PW_P3, 0);
	nrf24_Write_Register(RX_PW_P4, 0);
	nrf24_Write_Register(RX_PW_P5, 0);
	nrf24_Write_Register(FIFO_STATUS, 0x11);
	nrf24_Write_Register(DYNPD, 0);
	nrf24_Write_Register(FEATURE, 0);
	}
}

void NRF24_Init(){

	// Disable the chip before configuring the device
	CE_Disable();
	nrf24_reset(0);
	nrf24_Write_Register(CONFIG, 0);
	nrf24_Write_Register(EN_AA, 0);
	nrf24_Write_Register(EN_RXADDR, 0);
	nrf24_Write_Register(SETUP_AW, 0x03);  // 5 bytes for TX or RX address
	nrf24_Write_Register(SETUP_RETR, 0);   // No retransmission
	nrf24_Write_Register(RF_CH, 0);        // setup during TX or RX
	nrf24_Write_Register(RF_SETUP, 0x0E);  // 1110: LNA_HCURR can't change, PWR = 0dBm, DR = 2Mbps

	// Enable the chip after configuring the device
	CE_Enable();
}

// Setup TX mode
void NRF24_TxMode(uint8_t *Address, uint8_t channel){
	// Disable the chip before configuring the device
	CE_Disable();

	nrf24_Write_Register(RF_CH, channel); // select channel

	nrf24_Write_Register_Multiple(TX_ADDR, Address, 5);  // Ghi address Tx

	// Power up device by set bit 1 of CONFIG register = 1
	uint8_t config = nrf24_Read_Register(CONFIG);
	config |= (1<<1);
//	config = config & (0xF2);
	nrf24_Write_Register(CONFIG, config);
	// Enable the chip after configuring the device
	CE_Enable();
}

// Transmit data
uint8_t NRF24_Transmit(uint8_t *data){

	uint8_t cmdtosend = 0;
	//select device
	CS_Select();

	// payload command
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 1000);

	//send payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_Read_Register(FIFO_STATUS);
	// if 4th bit is set we assume send data successfully
	// 0x17(00010111) & (1<<4) = 00010000
	// 0x17 & (1<<3) = 00000000
	// notice: if disconnect device => all bit in FIFO_STATUS will be set, so if all bit is 0 assume successfully
	// nếu bit thứ 4 được set = 1 => truyền thành công
	// Trong trường hợp ngắt kết nối thiết bị giữa chừng thì chắc chắn k thể gửi thành công nên cần kiểm tra thêm
	// bằng cách: nếu tất cả các bit bằng 0 thì coi như kết nối toàn vẹn. Đó là bởi vì bit 2, 3 luôn bằng 0 nhưn
	// nếu mất kết nối thì sẽ được set = 1
	if((fifostatus & (1<<4)) && (!(fifostatus & (1<<3)))){
		cmdtosend = FLUSH_TX;
		nrf24_Send_Cmd(cmdtosend); // làm r?ng b? d?m (xóa)
		nrf24_reset (FIFO_STATUS);
		return 1;
	}
	return 0;
}

// Set up Rx mode
void NRF24_RxMode(uint8_t *Address, uint8_t channel){
	// Disable the chip before configuring the device
	CE_Disable();

	nrf24_reset (STATUS);
	nrf24_Write_Register(RF_CH, channel); // select channel

	// disable các giá tr? ban d?u c?a các pipe khác và ch?n 1 pipe

	uint8_t en_rxaddr = nrf24_Read_Register(EN_RXADDR);  // l?y giá tr? c?a thanh ghi EN_RXADDR
	en_rxaddr |= (1<<1); // set bit th? nh?t c?a thanh ghi dó lên 1 d? ch?n pipe 1;

	nrf24_Write_Register(EN_RXADDR, en_rxaddr); // ghi giá tr? value ban d?u c?a pipe 1 vào thanh ghi (ch?n pipe 1)


	// setup address c?a pipe
	nrf24_Write_Register_Multiple(RX_ADDR_P1, Address, 5);  // Ghi address cho pipe 1

	// setup data size cho data pipe (m?c d?nh là 32 bytes và k dùng ki?u dynamic byte)
	nrf24_Write_Register(RX_PW_P1, 32);

	// Power up device by set bit 0 and 1 of CONFIG register = 1
	uint8_t config = nrf24_Read_Register(CONFIG);
	config |= (1<<1)|(1<<0);
	nrf24_Write_Register(CONFIG, config);
	// Enable the chip after configuring the device
	CE_Enable();
}

// Ki?m tra xem có d? li?u ? b?t c? 1 pipe nào hay k
uint8_t isDataAvailable(int pipeNumber){
	uint8_t status = nrf24_Read_Register(STATUS);

	// Ki?m tra có d? li?u luu trong FIFO không hay là chu?n b? có d? li?u du?c luu vào hay k
	// ( t?c ki?m tra xem có d? li?u ? b?t c? pipe
	// nào trong 6 pipe hay là có d? li?u m?i chu?n b? t?i, n?u có set bit 6 lên 1)
	if((status & (1<<6) && (status & (pipeNumber<<1)))){
		nrf24_Write_Register(status, 1<<6);
		return 1;
	}
	return 0;
}

void NRF24_Receive(uint8_t *data){
	uint8_t cmdtosend = 0;
	//select device
	CS_Select();

	// payload command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 1000);

	//send payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrf24_Send_Cmd(cmdtosend);

	CS_UnSelect();
}