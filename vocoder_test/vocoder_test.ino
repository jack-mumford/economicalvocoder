#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── Pin Definitions ──────────────────────────────────────────
#define MIC_ADC_CHANNEL   ADC_CHANNEL_0   // GPIO36
#define DAC_PIN           25
//#define BUTTON1           15
#define BUTTON1 0
//#define BUTTON2           13
#define BUTTON2 35


// ── SD Card SPI Pins (HSPI, separate from TFT's VSPI) ───────
#define SD_MOSI           27
#define SD_MISO           33
#define SD_SCK            26
#define SD_CS             32

// ── Audio Parameters ─────────────────────────────────────────
#define SAMPLE_RATE       20000
#define FRAME_SIZE        400             // 20ms at 20kHz
#define LPC_ORDER         16

// ── ADC Continuous Config ────────────────────────────────────
#define ADC_FRAME_BYTES   (FRAME_SIZE * SOC_ADC_DIGI_RESULT_BYTES)
#define ADC_POOL_SIZE     (ADC_FRAME_BYTES * 4)
#define MAX_PARSED        (ADC_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES)

// ── State Machine ────────────────────────────────────────────
enum Mode { MODE_IDLE, MODE_VOCODER, MODE_RECORDING };
static Mode currentMode = MODE_IDLE;

TFT_eSPI tft = TFT_eSPI();
SPIClass sdSPI(HSPI);

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

static bool sdReady = false;

// Small per-frame output buffer for SD writes
static uint8_t frameOut[FRAME_SIZE];

// Recording file handle and sample counter
static File wavFile;
static uint32_t totalSamplesWritten = 0;
static int fileCounter = 0;

// ── Display Helpers ──────────────────────────────────────────
void displayStatus(const char* line1, const char* line2, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color);
  tft.setCursor(10, 40);
  tft.println(line1);
  if (line2) {
    tft.setCursor(10, 70);
    tft.println(line2);
  }
}

void displayIdle() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 30);
  tft.println("BTN1: VOCODER");
  tft.setCursor(10, 60);
  tft.println("BTN2: RECORD");
  if (!sdReady) {
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 100);
    tft.println("(No SD card)");
  }
}

// ── ADC ISR ──────────────────────────────────────────────────
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                        const adc_continuous_evt_data_t *edata,
                                        void *user_data) {
  BaseType_t mustYield = pdFALSE;
  vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
  return (mustYield == pdTRUE);
}

// ── ADC Init ─────────────────────────────────────────────────
bool initADCContinuous() {
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = ADC_POOL_SIZE,
    .conv_frame_size    = ADC_FRAME_BYTES,
  };
  esp_err_t err = adc_continuous_new_handle(&adc_config, &adc_handle);
  if (err != ESP_OK) {
    displayStatus("ADC handle fail:", esp_err_to_name(err), TFT_RED);
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
    displayStatus("ADC config fail:", esp_err_to_name(err), TFT_RED);
    return false;
  }

  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = adc_conv_done_cb,
  };
  err = adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
  if (err != ESP_OK) {
    displayStatus("ADC cb fail:", esp_err_to_name(err), TFT_RED);
    return false;
  }

  return true;
}

// ── SD Card Init ─────────────────────────────────────────────
bool initSD() {
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("SD card init failed");
    return false;
  }
  Serial.printf("SD card ready, size: %lluMB\n", SD.cardSize() / (1024 * 1024));
  return true;
}

// ── WAV Header Helpers ───────────────────────────────────────
// Write a placeholder header at file open, fix it on close
void writeWAVHeader(File &f, uint32_t numSamples) {
  uint32_t sampleRate = SAMPLE_RATE;
  uint16_t bitsPerSample = 8;
  uint16_t numChannels = 1;
  uint32_t byteRate = sampleRate * numChannels * bitsPerSample / 8;
  uint16_t blockAlign = numChannels * bitsPerSample / 8;
  uint32_t dataSize = numSamples;
  uint32_t chunkSize = 36 + dataSize;

  f.seek(0);
  f.write((const uint8_t*)"RIFF", 4);
  f.write((const uint8_t*)&chunkSize, 4);
  f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4);
  uint32_t fmtSize = 16;
  f.write((const uint8_t*)&fmtSize, 4);
  uint16_t audioFormat = 1;
  f.write((const uint8_t*)&audioFormat, 2);
  f.write((const uint8_t*)&numChannels, 2);
  f.write((const uint8_t*)&sampleRate, 4);
  f.write((const uint8_t*)&byteRate, 4);
  f.write((const uint8_t*)&blockAlign, 2);
  f.write((const uint8_t*)&bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4);
  f.write((const uint8_t*)&dataSize, 4);
}

// ── LPC Analysis (Levinson-Durbin) ───────────────────────────
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

// Forward declaration
void stopCurrentMode();

// ── Process Frame: VOCODER mode (DAC output) ─────────────────
void processFrameVocoder(int16_t* samples, int count) {
  for (int i = 0; i < count; i++) {
    dcEstimate += 0.001f * ((float)samples[i] - dcEstimate);
    floatFrame[i] = (float)samples[i] - dcEstimate;
  }

  for (int i = 0; i < count; i++) {
    floatFrame[i] /= 2048.0f;
  }

  float energy = 0;
  for (int i = 0; i < count; i++) {
    energy += floatFrame[i] * floatFrame[i];
  }
  energy = sqrtf(energy / count);

  // Noise gate
  if (energy < 0.02f) {
    for (int i = 0; i < count; i++) {
      dacWrite(DAC_PIN, 128);
      delayMicroseconds(46);
      if (digitalRead(BUTTON1) == LOW) {
        stopCurrentMode();
        return;
      }
    }
    return;
  }

  for (int i = 0; i < count; i++) {
    float w = 0.54f - 0.46f * cosf(2.0f * PI * i / (count - 1));
    windowed[i] = floatFrame[i] * w;
  }

  float lpc[LPC_ORDER];
  float gain;
  computeLPC(windowed, count, lpc, LPC_ORDER, &gain);

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

    if (digitalRead(BUTTON1) == LOW) {
      stopCurrentMode();
      return;
    }
  }
}

// ── Process Frame: RECORDING mode (SD card write) ────────────
// No DAC output, no timing delays — process as fast as possible
// and write each frame to the open WAV file
void processFrameRecord(int16_t* samples, int count) {
  for (int i = 0; i < count; i++) {
    dcEstimate += 0.001f * ((float)samples[i] - dcEstimate);
    floatFrame[i] = (float)samples[i] - dcEstimate;
  }

  for (int i = 0; i < count; i++) {
    floatFrame[i] /= 2048.0f;
  }

  float energy = 0;
  for (int i = 0; i < count; i++) {
    energy += floatFrame[i] * floatFrame[i];
  }
  energy = sqrtf(energy / count);

  // Noise gate — write silence
  if (energy < 0.02f) {
    memset(frameOut, 128, count);
    wavFile.write(frameOut, count);
    totalSamplesWritten += count;
    return;
  }

  for (int i = 0; i < count; i++) {
    float w = 0.54f - 0.46f * cosf(2.0f * PI * i / (count - 1));
    windowed[i] = floatFrame[i] * w;
  }

  float lpc[LPC_ORDER];
  float gain;
  computeLPC(windowed, count, lpc, LPC_ORDER, &gain);

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
    frameOut[i] = (uint8_t)constrain(dacVal, 0, 255);
  }

  // Write entire frame to SD at once
  wavFile.write(frameOut, count);
  totalSamplesWritten += count;
}

// ── Start / Stop ─────────────────────────────────────────────
void startMode(Mode mode) {
  currentMode = mode;
  frameIndex = 0;
  memset(filterState, 0, sizeof(filterState));
  carrierPhase = 0;
  dcEstimate = 2048.0f;

  if (mode == MODE_RECORDING) {
    // Open WAV file and write placeholder header
    char filename[32];
    snprintf(filename, sizeof(filename), "/vocoder_%03d.wav", fileCounter);
    wavFile = SD.open(filename, FILE_WRITE);
    if (!wavFile) {
      displayStatus("File open fail!", filename, TFT_RED);
      currentMode = MODE_IDLE;
      delay(1500);
      displayIdle();
      return;
    }
    totalSamplesWritten = 0;
    writeWAVHeader(wavFile, 0);  // placeholder, fixed on stop
    Serial.printf("Recording to %s\n", filename);
  }

  esp_err_t err = adc_continuous_start(adc_handle);
  if (err != ESP_OK) {
    displayStatus("ADC start fail:", esp_err_to_name(err), TFT_RED);
    if (mode == MODE_RECORDING) wavFile.close();
    currentMode = MODE_IDLE;
    return;
  }

  if (mode == MODE_VOCODER) {
    displayStatus("VOCODING...", "BTN1: stop", TFT_GREEN);
    Serial.println("Vocoder started");
  } else {
    displayStatus("RECORDING...", "BTN2: stop & save", TFT_RED);
    Serial.println("Recording started");
  }
}

void stopCurrentMode() {
  Mode wasMode = currentMode;
  currentMode = MODE_IDLE;

  adc_continuous_stop(adc_handle);
  dacWrite(DAC_PIN, 128);

  // Wait for button release to prevent immediate restart
  while (digitalRead(BUTTON1) == LOW || digitalRead(BUTTON2) == LOW) {
    delay(10);
  }
  delay(200);  // debounce

  if (wasMode == MODE_RECORDING && wavFile) {
    // Rewrite header with correct sample count
    displayStatus("Saving WAV...", NULL, TFT_YELLOW);
    writeWAVHeader(wavFile, totalSamplesWritten);
    wavFile.close();

    float duration = (float)totalSamplesWritten / SAMPLE_RATE;
    char buf1[32], buf2[32];
    snprintf(buf1, sizeof(buf1), "Saved: %.1fs", duration);
    snprintf(buf2, sizeof(buf2), "vocoder_%03d.wav", fileCounter);
    displayStatus(buf1, buf2, TFT_GREEN);
    Serial.printf("Saved vocoder_%03d.wav (%.1fs)\n", fileCounter, duration);
    fileCounter++;
    delay(2000);
  }

  displayIdle();
  Serial.println(wasMode == MODE_VOCODER ? "Vocoder stopped" : "Recording stopped");
}

// ── Arduino Setup ────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  tft.init();
  tft.setRotation(3);
  displayStatus("Initializing...", NULL, TFT_WHITE);

  // Init ADC
  s_task_handle = xTaskGetCurrentTaskHandle();
  if (!initADCContinuous()) {
    while (1) delay(1000);
  }

  // Init SD card
  sdReady = initSD();

  displayIdle();
  Serial.println("Vocoder + SD recorder ready");
}

// ── Arduino Loop ─────────────────────────────────────────────
void loop() {
  // IDLE: check buttons
  if (currentMode == MODE_IDLE) {
    if (digitalRead(BUTTON1) == LOW) {
      delay(200);
      startMode(MODE_VOCODER);
    }
    else if (digitalRead(BUTTON2) == LOW) {
      delay(200);
      if (!sdReady) {
        sdReady = initSD();
        if (!sdReady) {
          displayStatus("No SD card!", "Insert & retry", TFT_RED);
          delay(1500);
          displayIdle();
          return;
        }
        displayIdle();
        delay(200);
      }
      startMode(MODE_RECORDING);
    }
    else {
      delay(50);
    }
    return;
  }

  // ACTIVE: process audio
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
        if (currentMode == MODE_VOCODER) {
          processFrameVocoder(frameBuffer, FRAME_SIZE);
        } else if (currentMode == MODE_RECORDING) {
          processFrameRecord(frameBuffer, FRAME_SIZE);
        }
        frameIndex = 0;

        if (currentMode == MODE_IDLE) return;

        // Check stop button for recording between frames
        if (currentMode == MODE_RECORDING && digitalRead(BUTTON2) == LOW) {
          stopCurrentMode();
          return;
        }
      }
    }
  }
}