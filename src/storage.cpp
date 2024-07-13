#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "SPI.h" // SPI interface for SD card reader
#include "SD.h" // SD card reader
#include "Utilities.hpp" // Custom utilities for the project
#include "MavlinkUtilities.hpp"

SemaphoreHandle_t fileMutex;
const char* fileName = "/data.txt";
bool flashCardReady = false;

bool WriteFile(const String &filename, const String &content) {
    if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) { // Acquire the lock before accessing the file
        File file = SD.open(filename, FILE_APPEND);
        if (!file) {
            file = SD.open(filename, FILE_WRITE);
        }
        if (file) {
            file.print(content);
            file.print('\n');
            file.close();
            xSemaphoreGive(fileMutex); // Release the lock after accessing the file
            return true;
        } else {
            Serial.printf("\n[SD]Error opening file %s\n", filename.c_str());
            xSemaphoreGive(fileMutex); // Release the lock in case of error
            return false;
        }
    } else {
        Serial.println("Failed to acquire file mutex");
        return false;
    }
}

bool PrintFile(const String &filename) {
    if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) { // Acquire the lock before accessing the file
        File file = SD.open(filename);
        if (file) {
            while (file.available()) {
                Serial.write(file.read());
            }
            file.close();
            xSemaphoreGive(fileMutex); // Release the lock after accessing the file
            return true;
        } else {
            Serial.printf("\n[SD]Error opening file %s\n", filename.c_str());
            xSemaphoreGive(fileMutex); // Release the lock in case of error
            return false;
        }
    } else {
        Serial.println("Failed to acquire file mutex");
        return false;
    }
}

bool ReadFile(const String &filename, int numLines, String& buffer) {
    if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) { // Acquire the lock before accessing the file
        File file = SD.open(filename);
        if (file) {
            for (int i = 0; i < numLines && file.available(); i++) {
                buffer += file.readStringUntil('\n');
                buffer += '\n'; 
            }
            file.close();
            xSemaphoreGive(fileMutex); // Release the lock after accessing the file
            return true;
        } else {
            Serial.printf("\n[SD]Error opening file %s\n", filename.c_str());
            xSemaphoreGive(fileMutex); // Release the lock in case of error
            return false;
        }
    } else {
        Serial.println("Failed to acquire file mutex");
        return false;
    }
}

bool DeleteFile(const String &filename) {
    if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(10000)) == pdTRUE) {
        if (SD.remove(filename)) {
            xSemaphoreGive(fileMutex);
            Serial.printf("\n[SD]File %s deleted\n", filename.c_str());
            return true;
        } else {
            xSemaphoreGive(fileMutex);
            Serial.printf("\n[SD]Error deleting file %s\n", filename.c_str());
            return false;
        }
    } else {
        Serial.println("[SD]Failed to acquire file mutex for file deletion");
        return false;
    
    }
}

//Delete certain number of lines by reading them, writing to a new file, then deleting the old file and renaming the new file
bool DeleteFileLines(const String &fileName, int numberLinesToDelete) {

    if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(10000)) == pdTRUE) {
        File file = SD.open(fileName);
        if (!file) {
            Serial.printf("\n[SD]Error opening file %s\n", fileName.c_str());
            xSemaphoreGive(fileMutex);
            return false;
        }

        File newFile = SD.open("/temp.txt", FILE_WRITE);
        if (!newFile) {
            Serial.printf("\n[SD]Error creating temp file\n");
            file.close();
            xSemaphoreGive(fileMutex);
            return false;
        }

        int linesDeleted = 0;
        while (file.available() && linesDeleted < numberLinesToDelete) {
            String line = file.readStringUntil('\n');
            if (line.length() > 0) linesDeleted++;
        }

        while (file.available()) {
            String line = file.readStringUntil('\n');
            if (line.length() > 0) {
                newFile.print(line);
                newFile.print('\n');
            }
        }

        file.close();
        newFile.close();

        if (!SD.remove(fileName)) {
            Serial.printf("\n[SD]Error deleting file %s\n", fileName.c_str());
            xSemaphoreGive(fileMutex);
            return false;
        }

        if (!SD.rename("/temp.txt", fileName)) {
            Serial.printf("\n[SD]Error renaming file\n");
            xSemaphoreGive(fileMutex);
            return false;
        }

        xSemaphoreGive(fileMutex);
        return true;

    } else {
        Serial.println("[SD]Failed to acquire file mutex for file deletion");
        return false;
    }
}

bool FileHasContent(const String &filename) {
    if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) { // Acquire the lock before accessing the file
        File file = SD.open(filename);
        if (file) {
            bool hasContent = file.available();
            file.close();
            xSemaphoreGive(fileMutex); // Release the lock after accessing the file
            return hasContent;
        } else {
            Serial.printf("\n[SD]Error opening file %s\n", filename.c_str());
            xSemaphoreGive(fileMutex); // Release the lock in case of error
            return false;
        }
    } else {
        Serial.println("Failed to acquire file mutex");
        return false;
    }
}

bool SaveMavlinkMessage(mavlink_message_t message) {
    if (!flashCardReady) {
        DEBUG_PRINTF("\n[SD]Flash card not ready\n", NULL);
        return false;
    }
    String line = MavlinkToLineProtocol(message);
    if (line.isEmpty()) return false;
    return WriteFile(fileName, line);
}

static void serialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {

    const char* command = (const char*)event_data;

    if (strcmp(command, "delete") == 0) {
        if (DeleteFile(fileName)) {
            Serial.printf("\n[SD]File deleted\n");
        }
    }

    if (strcmp(command, "read") == 0) {
        String buffer;
        if (ReadFile(fileName, 10, buffer)) {
            Serial.printf("\n[SD]File content:\n%s\n", buffer.c_str());
        }
    }

    if (strcmp(command, "print") == 0) {
        if (PrintFile(fileName)) {
            Serial.printf("\n[SD]File printed\n");
        }
    }

    if (strcmp(command, "save") == 0) {
        mavlink_message_t message;
        if (!GenerateMavlinkMessage(message, MAVLINK_MSG_ID_ALL_INFO)) {
            Serial.printf("\n[SD]Failed to generate mavlink message\n");
            return;
        }
        if (SaveMavlinkMessage(message)) {
            Serial.printf("\n[SD]Mavlink message saved\n");
        }
    }
}

void TestMavlinkSave() {
    Serial.printf("\n[SD]Testing mavlink save\n");
    mavlink_message_t message;
    if (!GenerateMavlinkMessage(message, MAVLINK_MSG_ID_ALL_INFO)) {
        Serial.printf("\n[SD]Failed to generate mavlink message\n");
    }
    String line = MavlinkToLineProtocol(message);
    if (line.isEmpty()) return;
    Serial.printf("\n[SD]Line: %s\n", line.c_str());
}

/// @brief Access the SDCard to store measurements when internet connection is not available. Then send the measurements to the server when connection is restored.
/// @param parameter 
void FlashCardReaderTask(void *parameter) {

    //Create a mutex for the file system
    while ((fileMutex = xSemaphoreCreateMutex()) == NULL) {
        Serial.println("Failed to create file mutex");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    SPIClass spi = SPIClass(HSPI);
    spi.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);

    while (!SD.begin(SDCARD_CS, spi)) {
        Serial.println("[SD]Card Mount Failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    flashCardReady = true;
    Serial.println("[SD]Card Mount Success");

    // Register the serial command callback
    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, serialCommandCallback, NULL);


    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
