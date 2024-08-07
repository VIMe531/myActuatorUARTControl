# Makefile for building test.cpp with cl on Windows

# Compiler
CC = cl

# Compiler flags
CFLAGS = /EHsc

# Target executable
TARGET = test.exe

# Source file
SRC = UART_Motor_control_test.cpp

# Default rule
all: $(TARGET)

# Link rule
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) /Fe$(TARGET)

# Clean rule
clean:
	del $(TARGET)
