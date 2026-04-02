#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include <LittleFS.h>

class LittleFSLogger : public Print {
private:
    String buffer;

public:
    void rotate() {
        if (!LittleFS.exists("/system.log")) return;

        // Perform cleanup BEFORE appending to avoid exceeding limits
        checkRetention();

        // Append system.log to system.log.old
        File dest = LittleFS.open("/system.log.old", "a");
        File src = LittleFS.open("/system.log", "r");
        if (dest && src) {
            dest.println("\n--- [Rotation/Reboot] ---");
            uint8_t buf[256];
            while (src.available()) {
                size_t n = src.read(buf, sizeof(buf));
                dest.write(buf, n);
            }
        }
        if (dest) dest.close();
        if (src) src.close();

        LittleFS.remove("/system.log");
    }

    void flushToFile() {
        if (buffer.length() == 0) return;
        
        File f = LittleFS.open("/system.log", "a");
        if (f) {
            // If the current log file hits 16KB, merge it into the archive (.old)
            if (f.size() > 16384) { 
                f.close();
                rotate();
                f = LittleFS.open("/system.log", "a");
            }
            if (f) {
                f.print(buffer);
                f.close();
            }
        }
        buffer = "";
    }

public:
    void checkRetention() {
        if (!LittleFS.exists("/system.log.old")) return;

        bool wipe = false;
        File f = LittleFS.open("/system.log.old", "r");
        if (f) {
            // Size limit: 64KB
            if (f.size() > 65536) {
                wipe = true;
            }
            // Age limit: 7 days
            time_t lastWrite = f.getLastWrite();
            time_t now = time(nullptr);
            // Only check age if time is synced (later than 2020)
            if (now > 1577836800 && lastWrite > 0) {
                if (now - lastWrite > 7 * 24 * 3600) {
                    wipe = true;
                }
            }
            f.close();
        }

        if (wipe) {
            LittleFS.remove("/system.log.old");
        }
    }
    size_t write(uint8_t c) override {
        buffer += (char)c;
        if (c == '\n') {
            flushToFile(); // Write line chunk atomically
        }
        return 1;
    }
    
    size_t write(const uint8_t *buf, size_t size) override {
        for(size_t i = 0; i < size; i++) { 
            write(buf[i]); 
        }
        return size;
    }
};

extern LittleFSLogger fsLogger;
