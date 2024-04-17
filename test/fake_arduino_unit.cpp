#include <gtest/gtest.h>

#include "./faking/Arduino.h"

TEST(fakeArduino, sq) {
  ASSERT_EQ(sq(0), 0);
  ASSERT_EQ(sq(2), 4);
  ASSERT_EQ(sq(-3), 9);
}

TEST(fakeArduino, StringFromstdString) {
  ASSERT_EQ(String("abc"), "abc");
}

TEST(fakeArduino, StringFromDouble) {
  ASSERT_EQ(String(32.5), "32.5");
}

TEST(fakeArduino, StringToFloatInt) {
  ASSERT_EQ(String("23").toFloat(), 23);
}

TEST(fakeArduino, StringToFloatFloat) {
  ASSERT_NEAR(String("23.57").toFloat(), 23.57, 1e-6);
}

TEST(fakeArduino, StringToFloatError) {
  ASSERT_THROW(String("abc").toFloat(), std::invalid_argument);
}

TEST(fakeArduino, StringToFloatErrorSim) {
  ASSERT_EQ(String("15abc").toFloat(), 15);
}

TEST(fakeArduino, removeZerosIntNoZero) {
  ASSERT_EQ(_removeZeros("14"), "14");
}

TEST(fakeArduino, removeZerosFloatNoZero) {
  ASSERT_EQ(_removeZeros("14.67"), "14.67");
}

TEST(fakeArduino, removeZerosIntEndingWithZero) {
  ASSERT_EQ(_removeZeros("140"), "140");
}

TEST(fakeArduino, removeZerosFloatEndingWithZero) {
  ASSERT_EQ(_removeZeros("14.6700"), "14.67");
}

TEST(fakeArduino, removeZerosFloatWhichIsInt) {
  ASSERT_EQ(_removeZeros("14.0"), "14.0");
}

TEST(fakeArduino, removeZerosFloatWhichIsIntMultipleZero) {
  ASSERT_EQ(_removeZeros("14.000"), "14.0");
}

TEST(fakeArduino, removeZerosIntZero) {
  ASSERT_EQ(_removeZeros("0"), "0");
}

TEST(fakeArduino, removeZerosFloatZero) {
  ASSERT_EQ(_removeZeros("0.000"), "0.0");
}

TEST(fakeArduino, serialClassReadOnly) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  ASSERT_EQ(serial.read(), 't');
}

TEST(fakeArduino, serialClassMultipleReadOnly) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  serial.debugWrite('e');
  serial.debugWrite('s');
  serial.debugWrite('t');
  ASSERT_EQ(serial.read(), 't');
  ASSERT_EQ(serial.read(), 'e');
  ASSERT_EQ(serial.read(), 's');
  ASSERT_EQ(serial.read(), 't');
}

TEST(fakeArduino, serialClassWriteOnly) {
  SerialClass serial = SerialClass();
  serial.write('t');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassMultipleWriteOnly) {
  SerialClass serial = SerialClass();
  serial.write('t');
  serial.write('e');
  serial.write('s');
  serial.write('t');
  ASSERT_EQ(serial.debugRead(), 't');
  ASSERT_EQ(serial.debugRead(), 'e');
  ASSERT_EQ(serial.debugRead(), 's');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassReadWrite) {
  SerialClass serial = SerialClass();
  serial.debugWrite('d');
  serial.write('r');
  ASSERT_EQ(serial.read(), 'd');
  ASSERT_EQ(serial.debugRead(), 'r');
}

TEST(fakeArduino, serialClassMultipleReadWrite) {
  SerialClass serial = SerialClass();
  serial.write('t');
  serial.debugWrite('d');
  serial.write('e');
  serial.write('s');
  serial.debugWrite('f');
  serial.write('t');
  serial.debugWrite('g');
  serial.debugWrite('h');
  ASSERT_EQ(serial.read(), 'd');
  ASSERT_EQ(serial.debugRead(), 't');
  ASSERT_EQ(serial.debugRead(), 'e');
  ASSERT_EQ(serial.read(), 'f');
  ASSERT_EQ(serial.read(), 'g');
  ASSERT_EQ(serial.debugRead(), 's');
  ASSERT_EQ(serial.read(), 'h');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassPrintln) {
  SerialClass serial = SerialClass();
  serial.println("tes");
  serial.debugPrintln("df");
  serial.write('t');
  serial.debugPrintln("gh");
  ASSERT_EQ(serial.read(), 'd');
  ASSERT_EQ(serial.debugRead(), 't');
  ASSERT_EQ(serial.debugRead(), 'e');
  ASSERT_EQ(serial.read(), 'f');
  ASSERT_EQ(serial.read(), '\n');
  ASSERT_EQ(serial.read(), 'g');
  ASSERT_EQ(serial.debugRead(), 's');
  ASSERT_EQ(serial.read(), 'h');
  ASSERT_EQ(serial.debugRead(), '\n');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassAvailable) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  ASSERT_TRUE(serial.available());
}

TEST(fakeArduino, serialClassNotAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_FALSE(serial.available());
}

TEST(fakeArduino, serialClassAvailableAfterReading) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  serial.read();
  ASSERT_FALSE(serial.available());
}

TEST(fakeArduino, serialClassDebugAvailable) {
  SerialClass serial = SerialClass();
  serial.write('t');
  ASSERT_TRUE(serial.debugAvailable());
}

TEST(fakeArduino, serialClassNotDebugAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_FALSE(serial.debugAvailable());
}

TEST(fakeArduino, serialClassDebugAvailableAfterReading) {
  SerialClass serial = SerialClass();
  serial.write('t');
  serial.debugRead();
  ASSERT_FALSE(serial.debugAvailable());
}

TEST(fakeArduino, serialClassReadWhenNotAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_EQ(serial.read(), -1);
}

TEST(fakeArduino, serialClassReadWhenNotDebugAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_EQ(serial.debugRead(), -1);
}