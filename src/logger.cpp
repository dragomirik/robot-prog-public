#include "logger.h"

File _logFile;
unsigned int _logLevel;

void setupLog(int logLevel) {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("logger.setupLog : Erreur lors de l'initialisation de la carte microSD !, maybe there is no SD card, logger desactivated");
    return;
  }
  if (_logFile) {
    log_a(ErrorLevel, "logger.setupLog", "already setup");
  } else {
    _logFile = SD.open("log1.log", FILE_WRITE);
    _logLevel = logLevel;
    if (!_logFile) {
      SerialDebug.println("logger.setupLog : cannot open a file, maybe there is no SD card, logger desactivated");
    }
  }
  
}

void log_a(unsigned int level, String fromFun, String message) {
  if (_logFile && level >= _logLevel) {
    _logFile.println(fromFun + " : " + message);
    _logFile.flush();
  }
}

