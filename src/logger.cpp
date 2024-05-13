#include "logger.h"

void setupLog(int logLevel) {
  if (_logFile) {
    log_a(ErrorLevel, "logger.setupLog", "already setup");
  } else {
    _logFile = SD.open("log1.log");
    _logLevel = logLevel;
    if (!_logFile) {
      log_a(CriticalLevel, "logger.setupLog", "cannot open a file, maybe there is no SD card, logger desactivated");
    }
  }
  
}

void log_a(unsigned int level, String fromFun, String message) {
  if (_logFile && level >= _logLevel) {
    _logFile.println(fromFun + " : " + message);
  }
}

