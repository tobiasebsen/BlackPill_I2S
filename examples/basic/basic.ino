#include <BlackPill_I2S.h>

BlackPill_I2S I2S;

float osc_freq = 440;
float osc_delta = 0;
float osc_phase = 0;

void MyCallback(void* rx_data, void* tx_data, uint16_t size) {
  int16_t * in = (int16_t*)rx_data;
  int16_t * out = (int16_t*)tx_data;
  for (size_t i = 0; i < size; i+=2) {
    out[i+0] = out[i+1] = sinf(osc_phase) * 32000;
    osc_phase += osc_delta;
    if (osc_phase > TWO_PI)
      osc_phase -= TWO_PI;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  I2S.init(48000);
  I2S.begin(MyCallback);

  osc_delta = (osc_freq * TWO_PI) / I2S.get_samplerate();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(100);
}
