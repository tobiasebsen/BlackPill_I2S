#include <BlackPill_I2S.h>
#include "i2s_conf.h"

static I2S_Callback _callback;
static BlackPill_I2S * _i2s;

BlackPill_I2S::BlackPill_I2S() {
	_i2s = this;

	hi2s.Instance = I2S_SPI;

	hi2s.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s.Init.AudioFreq = I2S_AUDIOFREQ_48K;
	hi2s.Init.CPOL = I2S_CPOL_LOW;
	hi2s.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
}

bool BlackPill_I2S::init() {

	rcc_config();
	gpio_init();

	if (HAL_I2S_Init(&hi2s) != HAL_OK) {
		error_handler();
		return false;
	}

	return true;
}

bool BlackPill_I2S::init(I2S_InitTypeDef * i2s_init) {
	hi2s.Init = *i2s_init;
	return init();
}

bool BlackPill_I2S::init(uint32_t audio_freq, uint32_t bits, bool rx_enable, bool tx_enable, bool mck_enable, bool master) {
	if (master)
		hi2s.Init.Mode = tx_enable ? I2S_MODE_MASTER_TX : I2S_MODE_MASTER_RX;
	else
		hi2s.Init.Mode = tx_enable ? I2S_MODE_SLAVE_TX : I2S_MODE_SLAVE_RX;

	switch (bits){
	case 24: hi2s.Init.DataFormat = I2S_DATAFORMAT_24B; break;
	case 32: hi2s.Init.DataFormat = I2S_DATAFORMAT_32B; break;
	default: hi2s.Init.DataFormat = I2S_DATAFORMAT_16B;
	}

	hi2s.Init.MCLKOutput = mck_enable ? I2S_MCLKOUTPUT_ENABLE : I2S_MCLKOUTPUT_DISABLE;
	hi2s.Init.AudioFreq = audio_freq;
	hi2s.Init.FullDuplexMode = rx_enable && tx_enable ? I2S_FULLDUPLEXMODE_ENABLE : I2S_FULLDUPLEXMODE_DISABLE;

	return init();
}

bool BlackPill_I2S::deinit() {
	return HAL_I2S_DeInit(&hi2s) == HAL_OK;
}

void BlackPill_I2S::begin(I2S_Callback callback, uint16_t block_size) {
	uint16_t frame_size = get_bytes();
	uint16_t channels = 2;
	uint16_t *dma_buffer_tx = (uint16_t*)calloc(block_size * channels, frame_size);
	uint16_t *dma_buffer_rx = (uint16_t*)calloc(block_size * channels, frame_size);
	_callback = callback;

	if (hi2s.Init.FullDuplexMode == I2S_FULLDUPLEXMODE_ENABLE)
		transmit_receive_dma(dma_buffer_tx, dma_buffer_rx, block_size * channels);
	else if (hi2s.Init.Mode == I2S_MODE_MASTER_TX || hi2s.Init.Mode == I2S_MODE_SLAVE_TX)
		transmit_dma(dma_buffer_tx, block_size * channels);
	else if (hi2s.Init.Mode == I2S_MODE_MASTER_RX || hi2s.Init.Mode == I2S_MODE_SLAVE_RX)
		receive_dma(dma_buffer_rx, block_size * channels);
}

float BlackPill_I2S::get_samplerate() {

	uint32_t clk =  HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S);
	uint8_t bits = hi2s.Init.DataFormat == I2S_DATAFORMAT_16B ? 16 : 32;
	uint8_t i2sdiv = (hi2s.Instance->I2SPR & SPI_I2SPR_I2SDIV_Msk) >> SPI_I2SPR_I2SDIV_Pos;
	uint8_t odd = (hi2s.Instance->I2SPR & SPI_I2SPR_ODD_Msk) >> SPI_I2SPR_ODD_Pos;
	uint8_t mckoe = (hi2s.Instance->I2SPR & SPI_I2SPR_MCKOE_Msk) >> SPI_I2SPR_MCKOE_Pos;

	float fs = 0;

	if (mckoe) {
		if (bits == 16)
			 fs = clk / (float)((16*2)*((2*i2sdiv)+odd)*8);
		if (bits == 32)
			fs = clk / (float)((32*2)*((2*i2sdiv)+odd)*4);
	}
	else {
		if (bits == 16)
			 fs = clk / (float)((16*2)*((2*i2sdiv)+odd));
		if (bits == 32)
			fs = clk / (float)((32*2)*((2*i2sdiv)+odd));
	}

	return fs;
}

uint32_t BlackPill_I2S::get_bits() {
	switch (hi2s.Init.DataFormat) {
		case I2S_DATAFORMAT_16B: return 16;
		case I2S_DATAFORMAT_16B_EXTENDED: return 32;
		case I2S_DATAFORMAT_24B: return 24;
		case I2S_DATAFORMAT_32B: return 32;
	}
	return 0;
}

uint32_t BlackPill_I2S::get_bytes() {
	return (hi2s.Init.DataFormat == I2S_DATAFORMAT_16B) ? sizeof(int16_t) : sizeof(int32_t);
}

uint32_t BlackPill_I2S::get_blocksize() {
	return hi2s.TxXferSize > hi2s.RxXferSize ? hi2s.TxXferSize / 2 : hi2s.RxXferSize / 2;
}

bool BlackPill_I2S::transmit(uint16_t *data, uint16_t size, uint32_t timeout) {
	return HAL_I2S_Transmit(&hi2s, data, size, timeout) == HAL_OK;
}

bool BlackPill_I2S::receive(uint16_t *data, uint16_t size, uint32_t timeout) {
	return HAL_I2S_Receive(&hi2s, data, size, timeout) == HAL_OK;
}

bool BlackPill_I2S::transmit_it(uint16_t *data, uint16_t size) {
	return HAL_I2S_Transmit_IT(&hi2s, data, size) == HAL_OK;
}

bool BlackPill_I2S::receive_it(uint16_t *data, uint16_t size) {
	return HAL_I2S_Receive_IT(&hi2s, data, size) == HAL_OK;
}

bool BlackPill_I2S::transmit_dma(uint16_t *data, uint16_t size) {
	dma_init();
	return HAL_I2S_Transmit_DMA(&hi2s, data, size) == HAL_OK;
}

bool BlackPill_I2S::receive_dma(uint16_t *data, uint16_t size) {
	dma_init();
	return HAL_I2S_Receive_DMA(&hi2s, data, size) == HAL_OK;
}

bool BlackPill_I2S::transmit_receive_dma(uint16_t *tx_data, uint16_t *rx_data, uint16_t size) {
	dma_init();
	return HAL_I2SEx_TransmitReceive_DMA(&hi2s, tx_data, rx_data, size);
}


bool BlackPill_I2S::dma_pause() {
	return HAL_I2S_DMAPause(&hi2s);
}

bool BlackPill_I2S::dma_resume() {
	return HAL_I2S_DMAResume(&hi2s);
}

bool BlackPill_I2S::dma_stop() {
	return HAL_I2S_DMAStop(&hi2s);
}

bool BlackPill_I2S::rcc_config() {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	/** Initializes the peripherals clock
	 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#if defined(STM32F411xE)
    PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
#endif
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		error_handler();

	if (hi2s.Instance == SPI1)
		__HAL_RCC_SPI1_CLK_ENABLE();
	if (hi2s.Instance == SPI2)
		__HAL_RCC_SPI2_CLK_ENABLE();
	if (hi2s.Instance == SPI3)
		__HAL_RCC_SPI3_CLK_ENABLE();

	return true;
}

void BlackPill_I2S::gpio_init() {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	if (hi2s.Instance == SPI1) {
		gpio_pin_init(GPIOA, GPIO_PIN_5, GPIO_AF5_SPI1); // CK
		gpio_pin_init(GPIOA, GPIO_PIN_6, GPIO_AF5_SPI1); // WS
		gpio_pin_init(GPIOA, GPIO_PIN_7, GPIO_AF5_SPI1); // SD
	}
	if (hi2s.Instance == SPI2) {
		gpio_pin_init(GPIOB, GPIO_PIN_10, GPIO_AF5_SPI2); // CK
		gpio_pin_init(GPIOB, GPIO_PIN_12, GPIO_AF5_SPI2); // WS
		gpio_pin_init(GPIOB, GPIO_PIN_14, GPIO_AF6_I2S2ext); // ext_SD
		gpio_pin_init(GPIOB, GPIO_PIN_15, GPIO_AF5_SPI2); // SD
		if (hi2s.Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE)
			gpio_pin_init(GPIOA, GPIO_PIN_3, GPIO_AF5_SPI2); // MCK
	}
	if (hi2s.Instance == SPI3) {
		gpio_pin_init(GPIOB, GPIO_PIN_3, GPIO_AF6_SPI3); // CK
		gpio_pin_init(GPIOA, GPIO_PIN_4, GPIO_AF6_SPI3); // WS
		gpio_pin_init(GPIOB, GPIO_PIN_4, GPIO_AF7_I2S3ext); // ext_SD
		gpio_pin_init(GPIOB, GPIO_PIN_5, GPIO_AF6_SPI3); // SD
		if (hi2s.Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE)
			gpio_pin_init(GPIOB, GPIO_PIN_10, GPIO_AF6_SPI3); // MCK
	}
}

void BlackPill_I2S::gpio_pin_init(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t alternate) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = alternate;
	HAL_GPIO_Init(gpiox, &GPIO_InitStruct);
}

void BlackPill_I2S::gpio_pin_deinit(GPIO_TypeDef *gpiox, uint32_t pin) {
	HAL_GPIO_DeInit(gpiox, pin);
}

void BlackPill_I2S::gpio_mck_init() {
	if (hi2s.Instance == SPI2)
		gpio_pin_init(GPIOA, GPIO_PIN_3, GPIO_AF5_SPI2);
	if (hi2s.Instance == SPI3)
		gpio_pin_init(GPIOB, GPIO_PIN_10, GPIO_AF6_SPI3);
}

void BlackPill_I2S::gpio_ckin_init() {
	gpio_pin_init(GPIOA, GPIO_PIN_2, GPIO_AF5_SPI1);
}

void BlackPill_I2S::dma_init() {

	hdma_i2s_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_i2s_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2s_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_i2s_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_i2s_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_i2s_rx.Init.Mode = DMA_CIRCULAR;
	hdma_i2s_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_i2s_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_i2s_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	hdma_i2s_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_i2s_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	hdma_i2s_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_i2s_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2s_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_i2s_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_i2s_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_i2s_tx.Init.Mode = DMA_CIRCULAR;
	hdma_i2s_tx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_i2s_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_i2s_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	hdma_i2s_tx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_i2s_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	if (hi2s.Instance == SPI1) {
		__HAL_RCC_DMA2_CLK_ENABLE();
	}
	if (hi2s.Instance == SPI2) {
		__HAL_RCC_DMA1_CLK_ENABLE();
	}
	if (hi2s.Instance == SPI3) {
		__HAL_RCC_DMA1_CLK_ENABLE();
	}

#define DMA_IRQn_(a,b)		DMA##a##_Stream##b##_IRQn
#define DMA_IRQn(a,b)		DMA_IRQn_(a,b)
#define DMA_IRQn_RX			DMA_IRQn(I2S_DMAn,I2S_RX_STREAM)
#define DMA_IRQn_TX			DMA_IRQn(I2S_DMAn,I2S_TX_STREAM)

#define DMA_Stream_(a,b)	DMA##a##_Stream##b
#define DMA_Stream(a,b)		DMA_Stream_(a,b)
#define DMA_Stream_RX		DMA_Stream(I2S_DMAn,I2S_RX_STREAM)
#define DMA_Stream_TX		DMA_Stream(I2S_DMAn,I2S_TX_STREAM)

#define DMA_CHANNEL_N_(n)	DMA_CHANNEL_##n
#define DMA_CHANNEL_N(n)	DMA_CHANNEL_N_(n)
#define DMA_CHANNEL_RX		DMA_CHANNEL_N(I2S_RX_CHANNEL)
#define DMA_CHANNEL_TX		DMA_CHANNEL_N(I2S_TX_CHANNEL)

	if (hi2s.Init.FullDuplexMode == I2S_FULLDUPLEXMODE_ENABLE || hi2s.Init.Mode == I2S_MODE_MASTER_RX  || hi2s.Init.Mode == I2S_MODE_SLAVE_RX) {

		HAL_NVIC_SetPriority(DMA_IRQn_RX, 0, 0);
		HAL_NVIC_EnableIRQ(DMA_IRQn_RX);

		hdma_i2s_rx.Instance = DMA_Stream_RX;
		hdma_i2s_rx.Init.Channel = DMA_CHANNEL_RX;
		if (HAL_DMA_Init(&hdma_i2s_rx) != HAL_OK)
			error_handler();


		__HAL_LINKDMA(&hi2s, hdmarx, hdma_i2s_rx);
	}
	if (hi2s.Init.FullDuplexMode == I2S_FULLDUPLEXMODE_ENABLE || hi2s.Init.Mode == I2S_MODE_MASTER_TX || hi2s.Init.Mode == I2S_MODE_SLAVE_TX) {

		HAL_NVIC_SetPriority(DMA_IRQn_TX, 0, 0);
		HAL_NVIC_EnableIRQ(DMA_IRQn_TX);

		hdma_i2s_tx.Instance = DMA_Stream_TX;
		hdma_i2s_tx.Init.Channel = DMA_CHANNEL_TX;
		if (HAL_DMA_Init(&hdma_i2s_tx) != HAL_OK)
			error_handler();

		__HAL_LINKDMA(&hi2s, hdmatx, hdma_i2s_tx);
	}
}

void BlackPill_I2S::dma_irq_rx() {
	HAL_DMA_IRQHandler(&hdma_i2s_rx);
}

void BlackPill_I2S::dma_irq_tx() {
	HAL_DMA_IRQHandler(&hdma_i2s_tx);
}

void BlackPill_I2S::error_handler() {
	while (1);
}

#if I2S_USE_DMA

#define IRQ_HANDLER_(a,b) DMA##a##_Stream##b##_IRQHandler
#define IRQ_HANDLER(a,b) IRQ_HANDLER_(a, b)

extern "C" void IRQ_HANDLER(I2S_DMAn,I2S_RX_STREAM)() {
	_i2s->dma_irq_rx();
}

extern "C" void IRQ_HANDLER(I2S_DMAn,I2S_TX_STREAM)() {
	_i2s->dma_irq_tx();
}

void callback_half(I2S_HandleTypeDef *hi2s) {
  	uint8_t* buffer_rx = (uint8_t*)hi2s->pRxBuffPtr;
	uint8_t* buffer_tx = (uint8_t*)hi2s->pTxBuffPtr;
	uint16_t halfwords = hi2s->TxXferSize > hi2s->RxXferSize ? hi2s->TxXferSize : hi2s->RxXferSize;
	uint16_t size_samples = hi2s->Init.DataFormat == I2S_DATAFORMAT_16B ? halfwords : halfwords >> 1;
	_callback(buffer_rx, buffer_tx, size_samples / 2);
}

void callback_cplt(I2S_HandleTypeDef *hi2s) {
  	uint8_t* buffer_rx = (uint8_t*)hi2s->pRxBuffPtr;
	uint8_t* buffer_tx = (uint8_t*)hi2s->pTxBuffPtr;
	uint16_t halfwords = hi2s->TxXferSize > hi2s->RxXferSize ? hi2s->TxXferSize : hi2s->RxXferSize;
	uint16_t size_samples = hi2s->Init.DataFormat == I2S_DATAFORMAT_16B ? halfwords : halfwords >> 1;
	_callback(&buffer_rx[halfwords], &buffer_tx[halfwords], size_samples / 2);
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_half(hi2s);
}

extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_cplt(hi2s);
}

extern "C" void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_half(hi2s);
}

extern "C" void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_cplt(hi2s);
}

extern "C" void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_half(hi2s);
}

extern "C" void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_cplt(hi2s);
}

#endif