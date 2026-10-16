#define KEY_COUNT   5
const uint8_t PIN_KEY[KEY_COUNT] = { 2, 3, 4, 5, 6 };
#define PIN_LED    13

#define DEBOUNCE_MS    20
#define SCAN_MS         2
#define LONG_MS       800

#define SENDER_BAUD 115200

bool          stable[KEY_COUNT];
uint8_t       flipMs[KEY_COUNT];
unsigned long lastScanMs = 0;

bool          k5Hold      = false;
unsigned long k5DownMs    = 0;
bool          k5LongFired = false;

void sendKey(uint8_t n) {
  Serial.print('$');
  Serial.print('G');
  Serial.print((char)('0' + n));
  Serial.print('#');
}

void setup() {
  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    pinMode(PIN_KEY[i], INPUT_PULLUP);
    stable[i] = (digitalRead(PIN_KEY[i]) == LOW);
    flipMs[i] = 0;
  }
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(SENDER_BAUD);

  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(PIN_LED, HIGH); delay(120);
    digitalWrite(PIN_LED, LOW);  delay(120);
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastScanMs < SCAN_MS) {
    return;
  }
  lastScanMs = now;

  bool ledOn = false;

  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    bool raw = (digitalRead(PIN_KEY[i]) == LOW);

    if (raw != stable[i]) {
      if (flipMs[i] < DEBOUNCE_MS) {
        flipMs[i] += SCAN_MS;
      }
    } else {
      flipMs[i] = 0;
    }

    if (flipMs[i] >= DEBOUNCE_MS) {
      stable[i] = raw;
      flipMs[i] = 0;

      if (raw) {

        if (i == KEY_COUNT - 1) {
          k5Hold      = true;
          k5DownMs    = now;
          k5LongFired = false;
        } else {
          sendKey(i + 1);
        }
      } else {

        if (i == KEY_COUNT - 1 && k5Hold) {
          k5Hold = false;
          if (!k5LongFired) {
            sendKey(5);
          }
        }
      }
    }

    if (stable[i]) {
      ledOn = true;
    }
  }

  if (k5Hold && !k5LongFired && (now - k5DownMs) >= LONG_MS) {
    k5LongFired = true;
    sendKey(6);
  }

  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
}
