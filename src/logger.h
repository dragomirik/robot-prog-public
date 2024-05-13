#ifndef LOGGER_H
#define LOOGER_H

#include <Arduino.h>
#include <SD.h>
#include "utilities.h"

File _logFile;
unsigned int _logLevel;

const unsigned int NoteLevel = 10;
const unsigned int DebugLevel = 20;
const unsigned int InfoLevel = 30;
const unsigned int ErrorLevel = 40;
const unsigned int CriticalLevel = 50;

void setupLog(int logLevel);

void log_a(unsigned int level, String fromFun, String message);

#endif