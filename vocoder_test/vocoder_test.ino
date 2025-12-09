#include <SPI.h>
#include <TFT_eSPI.h>

#define MIC_PIN 36
#define DAC_PIN 25
#define BUTTON1 15
#define BUTTON2 13

#define SAMPLE_RATE 8000
#define RECORD_SECONDS 3
#define BUFFER_SIZE (SAMPLE_RATE * RECORD_SECONDS)

#define LPC_ORDER 16
#define FRAME_SIZE 160  // 20ms at 8kHz
#define HOP_SIZE 80     // 10ms hop

TFT_eSPI tft = TFT_eSPI();

int16_t inputBuffer[BUFFER_SIZE];
int16_t outputBuffer[BUFFER_SIZE];

void displayText(String text, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color);
  tft.setCursor(20, 60);
  tft.println(text);
}

void recordAudio() {
  for (int i = 3; i > 0; i--) {
    displayText("READY IN " + String(i), TFT_YELLOW);
    delay(1000);
  }
  
  displayText("RECORDING...\n3 SECONDS!", TFT_RED);
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    inputBuffer[i] = analogRead(MIC_PIN);
    delayMicroseconds(125);  // ~8kHz
  }
  
  displayText("READY", TFT_GREEN);
  Serial.println("Recording complete (3s)");
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
  if (E < 0.0001) E = 0.0001;
  
  for (int i = 0; i < order; i++) {
    lpc[i] = 0;
  }
  
  for (int i = 0; i < order; i++) {
    float lambda = 0;
    for (int j = 0; j < i; j++) {
      lambda += lpc[j] * autocorr[i - j];
    }
    lambda = (autocorr[i + 1] - lambda) / E;
    
    float temp[LPC_ORDER];
    for (int j = 0; j < i; j++) {
      temp[j] = lpc[j];
    }
    for (int j = 0; j < i; j++) {
      lpc[j] = temp[j] - lambda * temp[i - 1 - j];
    }
    lpc[i] = lambda;
    
    E = E * (1.0 - lambda * lambda);
    if (E < 0.0001) E = 0.0001;
  }
  
  *gain = sqrt(E);
}

void processVocoder() {
  displayText("PROCESSING...", TFT_YELLOW);
  Serial.println("LPC vocoder - monotone");
  
  long sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += inputBuffer[i];
  }
  float dcOffset = (float)sum / BUFFER_SIZE;
  
  float* floatInput = (float*)malloc(BUFFER_SIZE * sizeof(float));
  float peak = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    floatInput[i] = inputBuffer[i] - dcOffset;
    if (abs(floatInput[i]) > peak) peak = abs(floatInput[i]);
  }
  for (int i = 0; i < BUFFER_SIZE; i++) {
    floatInput[i] /= (peak + 1);
  }
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    outputBuffer[i] = 0;
  }
  
  float carrierFreq = 100.0;
  float carrierPhase = 0;
  
  float filterState[LPC_ORDER] = {0};
  
  float frame[FRAME_SIZE];
  float lpc[LPC_ORDER];
  float window[FRAME_SIZE];
  
  for (int i = 0; i < FRAME_SIZE; i++) {
    window[i] = 0.54 - 0.46 * cos(2.0 * PI * i / (FRAME_SIZE - 1));
  }
  
  int numFrames = (BUFFER_SIZE - FRAME_SIZE) / HOP_SIZE;
  
  for (int f = 0; f < numFrames; f++) {
    int frameStart = f * HOP_SIZE;
    
    for (int i = 0; i < FRAME_SIZE; i++) {
      frame[i] = floatInput[frameStart + i] * window[i];
    }
    
    float gain;
    computeLPC(frame, FRAME_SIZE, lpc, LPC_ORDER, &gain);
    
    float energy = 0;
    for (int i = 0; i < FRAME_SIZE; i++) {
      energy += frame[i] * frame[i];
    }
    energy = sqrt(energy / FRAME_SIZE);
    
    for (int i = 0; i < HOP_SIZE; i++) {
      int outIdx = frameStart + i;
      if (outIdx >= BUFFER_SIZE) break;
      
      float excitation = 0;
      carrierPhase += carrierFreq / SAMPLE_RATE;
      if (carrierPhase >= 1.0) {
        carrierPhase -= 1.0;
        excitation = 1.0;
      }
      
      excitation *= energy * 15.0;
      
      float output = excitation;
      for (int j = 0; j < LPC_ORDER; j++) {
        output += lpc[j] * filterState[j];
      }
      
      for (int j = LPC_ORDER - 1; j > 0; j--) {
        filterState[j] = filterState[j - 1];
      }
      filterState[0] = output;
      
      outputBuffer[outIdx] += (int16_t)(output * 2000);
    }
    
    if (f % 30 == 0) Serial.print(".");
  }
  
  free(floatInput);
  
  Serial.println("\nDone!");
  displayText("PRESS BTN2\nTO PLAY", TFT_CYAN);
}

void playbackAudio() {
  displayText("PLAYING...", TFT_GREEN);
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int dacVal = (outputBuffer[i] >> 8) + 128;
    dacWrite(DAC_PIN, constrain(dacVal, 0, 255));
    delayMicroseconds(125);  // ~8kHz
  }
  
  displayText("READY", TFT_GREEN);
  Serial.println("Playback complete");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  tft.init();
  tft.setRotation(3);
  
  displayText("BTN1: RECORD\nBTN2: PROCESS", TFT_WHITE);
  Serial.println("Ready! LPC vocoder (3s)");
}

void loop() {
  static bool hasRecording = false;
  
  if (digitalRead(BUTTON1) == LOW) {
    delay(200);
    recordAudio();
    hasRecording = true;
    delay(500);
  }
  
  if (digitalRead(BUTTON2) == LOW) {
    if (!hasRecording) {
      displayText("RECORD FIRST!", TFT_RED);
      delay(2000);
      displayText("BTN1: RECORD\nBTN2: PROCESS", TFT_WHITE);
    } else {
      delay(200);
      processVocoder();
      delay(1000);
      playbackAudio();
    }
    delay(500);
  }
}