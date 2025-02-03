
TARGET = NUCLEO_F446RE
TOOLCHAIN = GCC_ARM
CXXFLAGS += -Wno-enum-conversion

all : default

default:
	mbed-tools compile -m $(TARGET) -t GCC_ARM -f #Configure, compile, flash

compile:
	mbed-tools compile -m $(TARGET) -t GCC_ARM #Configure, compile, flash

clean_build:
	mbed-tools compile -m $(TARGET) -t GCC_ARM --clean #Configure, compile, flash