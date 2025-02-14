
TARGET = NUCLEO_F446RE
TOOLCHAIN = GCC_ARM
CXXFLAGS += -Wno-enum-conversion

all : default docs

default:
	mbed-tools compile -m $(TARGET) -t GCC_ARM -f #Configure, compile, flash

compile:
	mbed-tools compile -m $(TARGET) -t GCC_ARM #Configure, compile, flash

clean_build:
	mbed-tools compile -m $(TARGET) -t GCC_ARM --clean #Configure, compile, flash

docs:
	doxygen Doxyfile

minicom:
	minicom -D /dev/ttyACM0 -b 115200

clean_docs:
	rm -rf docs/html docs/latex

.PHONY: all default compile clean_build docs minicom clean_docs