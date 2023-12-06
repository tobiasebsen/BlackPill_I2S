#include <Arduino.h>

#include "stm32f4xx_hal.h"

typedef void (*I2S_Callback)(void* rx_data, void* tx_data, uint16_t size);

class BlackPill_I2S {
public:
	BlackPill_I2S();

	bool init();
	bool init(I2S_InitTypeDef * i2s_init);
	bool init(uint32_t audio_freq, uint32_t bits = 16, bool rx_enable = false, bool tx_enable = true, bool mck_enable = false, bool master = true);
	bool deinit();

	void begin(I2S_Callback callback, uint16_t block_size = 64);

	float get_samplerate();
	uint32_t get_bits();
	uint32_t get_bytes();
	uint32_t get_blocksize();

	bool transmit(uint16_t *data, uint16_t size, uint32_t timeout);
	bool receive(uint16_t *data, uint16_t size, uint32_t timeout);
	bool transmit_receive(uint16_t *tx_data, uint16_t *rx_data, uint16_t size, uint32_t timeout);

	bool transmit_it(uint16_t *data, uint16_t size);
	bool receive_it(uint16_t *data, uint16_t size);
	bool transmit_receive_it(uint16_t *tx_data, uint16_t *rx_data, uint16_t size);

	bool transmit_dma(uint16_t *data, uint16_t size);
	bool receive_dma(uint16_t *data, uint16_t size);
	bool transmit_receive_dma(uint16_t *tx_data, uint16_t *rx_data, uint16_t size);

	bool dma_pause();
	bool dma_resume();
	bool dma_stop();

	void dma_irq_rx();
	void dma_irq_tx();

	void gpio_mck_init();
	void gpio_ckin_init();

private:
	I2S_HandleTypeDef hi2s;
	DMA_HandleTypeDef hdma_i2s_tx;
	DMA_HandleTypeDef hdma_i2s_rx;

	bool rcc_config();

	void gpio_init();
	void gpio_pin_init(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t alternate);
	void gpio_pin_deinit(GPIO_TypeDef *gpiox, uint32_t pin);

	void dma_init();

	void error_handler();
};