#include <rgb.h>

void setup() {
    Serial.begin(115200);
    setupRGB();
}

void loop() {
    handleRGB();
}