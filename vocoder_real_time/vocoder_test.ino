#include <SPI.h>
#include <TFT_eSPI.h>
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MIC_ADC_CHANNEL   ADC_CHANNEL_0
#define DAC_PIN           25
#define BUTTON1           15
#define BUTTON2           13

#define SAMPLE_RATE       20000
#define FRAME_SIZE        400
#define LPC_ORDER         16

#define ADC_FRAME_BYTES   (FRAME_SIZE * SOC_ADC_DIGI_RESULT_BYTES)
#define ADC_POOL_SIZE     (ADC_FRAME_BYTES * 4)
#define MAX_PARSED        (ADC_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES)

TFT_eSPI tft = TFT_eSPI();

static adc_continuous_handle_t adc_handle = NULL;
static TaskHandle_t s_task_handle;

static int16_t frameBuffer[FRAME_SIZE];
static int frameIndex = 0;

static float floatFrame[FRAME_SIZE];
static float windowed[FRAME_SIZE];
static float filterState[LPC_ORDER] = {0};
static float carrierPhase = 0.0;
static float carrierFreq = 100.0;
static float dcEstimate = 2048.0;

static uint8_t rawBuf[ADC_FRAME_BYTES];
static adc_continuous_data_t parsedData[MAX_PARSED];

static bool vocoderActive = false;

void displayText(const char* text, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color);
  tft.setCursor(20, 60);
  tft.println(text);
}

static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                        const adc_continuous_evt_data_t *edata,
                                        void *user_data) {
  BaseType_t mustYield = pdFALSE;
  vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
  return (mustYield == pdTRUE);
}

bool initADCContinuous() {
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = ADC_POOL_SIZE,
    .conv_frame_size    = ADC_FRAME_BYTES,
  };
  esp_err_t err = adc_continuous_new_handle(&adc_config, &adc_handle);
  if (err != ESP_OK) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 10);
    tft.printf("new_handle fail:\n%s", esp_err_to_name(err));
    return false;
  }

  adc_digi_pattern_config_t adc_pattern[1] = {};
  adc_pattern[0].atten     = ADC_ATTEN_DB_12;
  adc_pattern[0].channel   = MIC_ADC_CHANNEL & 0x7;
  adc_pattern[0].unit      = ADC_UNIT_1;
  adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t dig_cfg = {
    .pattern_num    = 1,
    .adc_pattern    = adc_pattern,
    .sample_freq_hz = SAMPLE_RATE,
    .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
  };
  err = adc_continuous_config(adc_handle, &dig_cfg);
  if (err != ESP_OK) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 10);
    tft.printf("config fail:\n%s", esp_err_to_name(err));
    return false;
  }

  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = adc_conv_done_cb,
  };
  err = adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
  if (err != ESP_OK) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 10);
    tft.printf("callback fail:\n%s", esp_err_to_name(err));
    return false;
  }

  return true;
}

void computeLPC(float* frame, int frameSize, float* lpc, int order, float* gain) {
  float autocorr[LPC_ORDER + 1];

  for (int i = 0; i <= order; i++) {
    autocorr[i] = 0;
    for (int j = 0; j < frameSize - i; j++) {
      autocorr[i] += frame[j] * frame[j + i];
    }
  }

  float E = autocorr[0];
  if (E < 0.0001f) E = 0.0001f;

  for (int i = 0; i < order; i++) lpc[i] = 0;

  for (int i = 0; i < order; i++) {
    float lambda = 0;
    for (int j = 0; j < i; j++) {
      lambda += lpc[j] * autocorr[i - j];
    }
    lambda = (autocorr[i + 1] - lambda) / E;

    float temp[LPC_ORDER];
    for (int j = 0; j < i; j++) temp[j] = lpc[j];
    for (int j = 0; j < i; j++) lpc[j] = temp[j] - lambda * temp[i - 1 - j];
    lpc[i] = lambda;

    E *= (1.0f - lambda * lambda);
    if (E < 0.0001f) E = 0.0001f;
  }

  *gain = sqrtf(E);
}

void processAndOutputFrame(int16_t* samples, int count) {
  // DC removal and float conversion
  for (int i = 0; i < count; i++) {
    dcEstimate += 0.001f * ((float)samples[i] - dcEstimate);
    floatFrame[i] = (float)samples[i] - dcEstimate;
  }

  // Fixed scale instead of per-frame normalization
  // 12-bit ADC centered, so max swing is ~2048
  for (int i = 0; i < count; i++) {
    floatFrame[i] /= 2048.0f;
  }

  // Frame energy (before windowing)
  float energy = 0;
  for (int i = 0; i < count; i++) {
    energy += floatFrame[i] * floatFrame[i];
  }
  energy = sqrtf(energy / count);

  // Noise gate - output silence if frame is too quiet
  if (energy < 0.02f) {
    for (int i = 0; i < count; i++) {
      dacWrite(DAC_PIN, 128);
      delayMicroseconds(46);
      if (digitalRead(BUTTON2) == LOW) {
        vocoderActive = false;
        adc_continuous_stop(adc_handle);
        dacWrite(DAC_PIN, 128);
        displayText("BTN1: START\nBTN2: STOP", TFT_WHITE);
        Serial.println("Vocoder stopped");
        return;
      }
    }
    return;
  }

  // Hamming window for LPC analysis
  for (int i = 0; i < count; i++) {
    float w = 0.54f - 0.46f * cosf(2.0f * PI * i / (count - 1));
    windowed[i] = floatFrame[i] * w;
  }

  // LPC analysis
  float lpc[LPC_ORDER];
  float gain;
  computeLPC(windowed, count, lpc, LPC_ORDER, &gain);

  // Synthesize and output
  for (int i = 0; i < count; i++) {
    float excitation = 0;
    carrierPhase += carrierFreq / SAMPLE_RATE;
    if (carrierPhase >= 1.0f) {
      carrierPhase -= 1.0f;
      excitation = 1.0f;
    }
    excitation *= energy * 15.0f;

    float output = excitation;
    for (int j = 0; j < LPC_ORDER; j++) {
      output += lpc[j] * filterState[j];
    }

    for (int j = LPC_ORDER - 1; j > 0; j--) {
      filterState[j] = filterState[j - 1];
    }
    filterState[0] = output;

    int dacVal = (int)(output * 8.0f) + 128;
    dacWrite(DAC_PIN, constrain(dacVal, 0, 255));
    delayMicroseconds(46);

    if (digitalRead(BUTTON2) == LOW) {
      vocoderActive = false;
      adc_continuous_stop(adc_handle);
      dacWrite(DAC_PIN, 128);
      displayText("BTN1: START\nBTN2: STOP", TFT_WHITE);
      Serial.println("Vocoder stopped");
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(20, 60);
  tft.println("Initializing...");

  s_task_handle = xTaskGetCurrentTaskHandle();

  if (!initADCContinuous()) {
    Serial.println("ADC init failed - check screen");
    while (1) delay(1000);
  }

  displayText("BTN1: START\nBTN2: STOP", TFT_WHITE);
  Serial.println("Real-time LPC vocoder ready");
}

void loop() {
  if (digitalRead(BUTTON1) == LOW && !vocoderActive) {
    delay(200);
    vocoderActive = true;
    frameIndex = 0;
    memset(filterState, 0, sizeof(filterState));
    carrierPhase = 0;
    dcEstimate = 2048.0f;

    esp_err_t err = adc_continuous_start(adc_handle);
    if (err != ESP_OK) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(TFT_RED);
      tft.setCursor(10, 10);
      tft.printf("start fail:\n%s", esp_err_to_name(err));
      vocoderActive = false;
      return;
    }
    displayText("VOCODING...", TFT_GREEN);
    Serial.println("Vocoder started");
  }

  if (digitalRead(BUTTON2) == LOW && vocoderActive) {
    delay(200);
    vocoderActive = false;
    adc_continuous_stop(adc_handle);
    dacWrite(DAC_PIN, 128);
    displayText("BTN1: START\nBTN2: STOP", TFT_WHITE);
    Serial.println("Vocoder stopped");
  }

  if (!vocoderActive) {
    delay(50);
    return;
  }

  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

  uint32_t ret_num = 0;

  while (adc_continuous_read(adc_handle, rawBuf, ADC_FRAME_BYTES, &ret_num, 0) == ESP_OK) {
    uint32_t num_parsed = 0;

    esp_err_t err = adc_continuous_parse_data(adc_handle, rawBuf, ret_num, parsedData, &num_parsed);
    if (err != ESP_OK) continue;

    for (uint32_t i = 0; i < num_parsed; i++) {
      if (!parsedData[i].valid) continue;

      frameBuffer[frameIndex++] = (int16_t)parsedData[i].raw_data;

      if (frameIndex >= FRAME_SIZE) {
        processAndOutputFrame(frameBuffer, FRAME_SIZE);
        frameIndex = 0;
        if (!vocoderActive) return;
      }
    }
  }
}