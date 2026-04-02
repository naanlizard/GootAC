#include <Arduino.h>
#include <LittleFS.h>
void printFSInfo() {
    LittleFS.begin();
    FSInfo info;
    LittleFS.info(info);
    Serial.printf("FS Total: %d, Used: %d\n", info.totalBytes, info.usedBytes);
}
