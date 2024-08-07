#include <windows.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdint>

#define DATA_LENGTH 8

class CRC16 {
public:
    static uint16_t calculate(const uint8_t* data, size_t start, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = start; i < length; ++i) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0xA001;
                else
                    crc >>= 1;
            }
        }
        return crc;
    }
};

struct Serialmessage{
    uint8_t header = 0x3E;
    uint8_t ID;
    std::vector<uint8_t> data{8};
    uint8_t data_length;
    uint16_t crc;
};

void handleSerialPortError(const std::string& message) {
    std::cerr << message << std::endl;
}

void handleOpenError(HANDLE serialPort) {
    if(serialPort == INVALID_HANDLE_VALUE){
        handleSerialPortError("Error Opening to serial port");
        CloseHandle(serialPort);
        exit(1);
    }
    else{
        handleSerialPortError("Opening serial port successful");
    }
}

void handleWriteError(HANDLE serialPort, std::vector<uint8_t> message, DWORD bytes_written) {
    if(!WriteFile(serialPort, message.data(), message.size(), &bytes_written, NULL)){
        handleSerialPortError("Error Writing to serial port");
        CloseHandle(serialPort);
        exit(1);
    }
    else{
        handleSerialPortError("Writing to serial port has succeeded");
        std::cout << "Bytes written to serial port: " << bytes_written << std::endl;
        std::cout << "written data : ";
        for (DWORD i = 0; i < bytes_written; ++i) {
            std::cout << std::hex << static_cast<int>(message[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
}

uint32_t handleReadError(HANDLE serialPort, std::vector<uint8_t> message, DWORD total_bytes_read, DWORD bytes_read, uint32_t target) {
    while (total_bytes_read < message.size()) {
        if (!ReadFile(serialPort, message.data() + total_bytes_read, message.size() - total_bytes_read, &bytes_read, NULL)) {
            DWORD error = GetLastError();
            std::cerr << "ReadFile failed with error " << error << std::endl;
            CloseHandle(serialPort);
            exit(1);
        }
        else{
            total_bytes_read += bytes_read;
        }
    }
    if (total_bytes_read == message.size()) {
        std::cout << "Bytes read from serial port: " << total_bytes_read << std::endl;
        target = (static_cast<uint16_t>(message[10]) << 24) | (static_cast<uint16_t>(message[9]) << 16) | (static_cast<uint16_t>(message[8]) << 8) | static_cast<uint16_t>(message[7]);
        std::cout << "Received data: ";
        for (DWORD i = 0; i < total_bytes_read; ++i) {
            std::cout << std::hex << static_cast<int>(message[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
    else {
        handleSerialPortError("Failed to read complete message");
    }
    return target;
}

void handleTimeoutsError(HANDLE serialPort, COMMTIMEOUTS timeouts) {
    if (!SetCommTimeouts(serialPort, &timeouts)) {
        handleSerialPortError("Error setting timeouts");
        CloseHandle(serialPort);
        exit(1);
    }
    else;
}

void handleCommStateError(HANDLE serialPort, DCB dcbSerialParams) {
    if(!GetCommState(serialPort, &dcbSerialParams)){
        handleSerialPortError("Error getting serial port state");
        CloseHandle(serialPort);
        exit(1);
    }
    else;
}

void handleSetCommStateError(HANDLE serialPort, DCB dcbSerialParams) {
    if (!SetCommState(serialPort, &dcbSerialParams)) {
        handleSerialPortError("Error setting serial port state");
        CloseHandle(serialPort);
        exit(1);
    }
    else;
}

uint32_t SerialCommu(LPCWSTR port_name, DCB dcbSerialParams, COMMTIMEOUTS timeouts, std::vector<uint8_t> message) {
    uint32_t target = 0x00000000;

    HANDLE serialPort = CreateFileW(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    DWORD bytes_written = 0, bytes_read = 0;
    DWORD total_bytes_read = 0;

    handleOpenError(serialPort);
    handleCommStateError(serialPort, dcbSerialParams);
    handleSetCommStateError(serialPort, dcbSerialParams);
    handleTimeoutsError(serialPort, timeouts);
    handleWriteError(serialPort, message, bytes_written);
    PurgeComm(serialPort, PURGE_RXCLEAR);
    target = handleReadError(serialPort, message, total_bytes_read, bytes_read, target);
    CloseHandle(serialPort);
    
    return target;
}

uint32_t SetAccel(uint8_t function_idx) {
    Serialmessage s_message;
    s_message.ID = 1;
    s_message.data[0] = 0x43;
    s_message.data[1] = function_idx;
    s_message.data[2] = 0x00;
    s_message.data[3] = 0x00;
    s_message.data[4] = 0x00;
    s_message.data[5] = 0x00;
    s_message.data[6] = 0x00;
    s_message.data[7] = 0x00;
    s_message.data_length = DATA_LENGTH;
    uint32_t Accel = 0x00000000;

    std::vector<uint8_t> message;
    message.push_back(s_message.header);
    message.push_back(s_message.ID);
    message.push_back(s_message.data_length);
    message.push_back(s_message.data[0]);
    message.push_back(s_message.data[1]);
    message.push_back(s_message.data[2]);
    message.push_back(s_message.data[3]);
    message.push_back(s_message.data[4]);
    message.push_back(s_message.data[5]);
    message.push_back(s_message.data[6]);
    message.push_back(s_message.data[7]);
    // re-calculate CRC
    uint16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    // Send data to COM port
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    Accel = SerialCommu(port_name, dcbSerialParams, timeouts, message);
    return Accel;
}

uint16_t ReadAccel(uint8_t function_idx) {
    Serialmessage s_message;
    s_message.ID = 1;
    s_message.data[0] = 0x42;
    s_message.data[1] = function_idx; //data index
    s_message.data[2] = 0x00;
    s_message.data[3] = 0x00;
    s_message.data[4] = 0x00;
    s_message.data[5] = 0x00;
    s_message.data[6] = 0x00;
    s_message.data[7] = 0x00;
    s_message.data_length = DATA_LENGTH;
    uint32_t Accel = 0x00000000;

    std::vector<uint8_t> message;
    message.push_back(s_message.header);
    message.push_back(s_message.ID);
    message.push_back(s_message.data_length);
    message.push_back(s_message.data[0]);
    message.push_back(s_message.data[1]);
    message.push_back(s_message.data[2]);
    message.push_back(s_message.data[3]);
    message.push_back(s_message.data[4]);
    message.push_back(s_message.data[5]);
    message.push_back(s_message.data[6]);
    message.push_back(s_message.data[7]);
    // re-calculate CRC
    uint16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    HANDLE serialPort;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    Accel = SerialCommu(port_name, dcbSerialParams, timeouts, message);
    return Accel;
}

uint16_t ReadSingleAngle(void) {
    Serialmessage s_message;
    s_message.ID = 1;
    s_message.data[0] = 0x94;
    s_message.data[1] = 0x00;
    s_message.data[2] = 0x00;
    s_message.data[3] = 0x00;
    s_message.data[4] = 0x00;
    s_message.data[5] = 0x00;
    s_message.data[6] = 0x00;
    s_message.data[7] = 0x00;
    s_message.data_length = DATA_LENGTH;
    uint16_t AbsAngle = 0x0000;

    std::vector<uint8_t> message;
    message.push_back(s_message.header);
    message.push_back(s_message.ID);
    message.push_back(s_message.data_length);
    message.push_back(s_message.data[0]);
    message.push_back(s_message.data[1]);
    message.push_back(s_message.data[2]);
    message.push_back(s_message.data[3]);
    message.push_back(s_message.data[4]);
    message.push_back(s_message.data[5]);
    message.push_back(s_message.data[6]);
    message.push_back(s_message.data[7]);
    // re-calculate CRC
    uint16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    HANDLE serialPort;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    serialPort = CreateFileW(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (serialPort == INVALID_HANDLE_VALUE) {
        std::cerr << "Error in opening serial port" << std::endl;
    } else {
        std::cout << "Opening serial port successful" << std::endl;

        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(serialPort, &dcbSerialParams)) {
            std::cerr << "Error getting serial port state" << std::endl;
        } else {
            dcbSerialParams.BaudRate = CBR_115200;
            dcbSerialParams.ByteSize = 8;
            dcbSerialParams.StopBits = ONESTOPBIT;
            dcbSerialParams.Parity = NOPARITY;
            if (!SetCommState(serialPort, &dcbSerialParams)) {
                std::cerr << "Error setting serial port state" << std::endl;
            } else {
                timeouts.ReadIntervalTimeout = 50;
                timeouts.ReadTotalTimeoutConstant = 50;
                timeouts.ReadTotalTimeoutMultiplier = 10;
                timeouts.WriteTotalTimeoutConstant = 50;
                timeouts.WriteTotalTimeoutMultiplier = 10;
                if (!SetCommTimeouts(serialPort, &timeouts)) {
                    std::cerr << "Error setting timeouts" << std::endl;
                } else {
                    DWORD bytes_written, bytes_read;
                    if (!WriteFile(serialPort, message.data(), message.size(), &bytes_written, NULL)) {
                        std::cerr << "Error writing to serial port" << std::endl;
                    } else {
                        std::cout << "Bytes written to serial port: " << bytes_written << std::endl;
                        std::cout << "written data : ";
                        for (DWORD i = 0; i < bytes_written; ++i) {
                            std::cout << std::hex << static_cast<int>(message[i]) << " ";
                        }
                        std::cout << std::dec << std::endl;
                        
                        PurgeComm(serialPort, PURGE_RXCLEAR);
                        
                        DWORD total_bytes_read = 0;
                        while (total_bytes_read < message.size()) {
                            if (!ReadFile(serialPort, message.data() + total_bytes_read, message.size() - total_bytes_read, &bytes_read, NULL)) {
                                DWORD error = GetLastError();
                                std::cerr << "ReadFile failed with error " << error << std::endl;
                                break;
                            }
                            total_bytes_read += bytes_read;
                        }

                        if (total_bytes_read == message.size()) {
                            std::cout << "Bytes read from serial port: " << total_bytes_read << std::endl;
                            AbsAngle = (static_cast<uint16_t>(message[10]) << 8) | static_cast<uint16_t>(message[9]);
                            std::cout << "Received data: ";
                            for (DWORD i = 0; i < total_bytes_read; ++i) {
                                std::cout << std::hex << static_cast<int>(message[i]) << " ";
                            }
                            std::cout << std::dec << std::endl;
                        } else {
                            std::cerr << "Failed to read complete message" << std::endl;
                        }
                    }
                }
            }
        }
        CloseHandle(serialPort);
    }
    return AbsAngle;
}


int main(void) {

    uint16_t Angle = ReadSingleAngle();
    uint32_t get_Accel = ReadAccel(0x00);
    uint32_t in_Accel = SetAccel(0x00);
    std::cout << "Motor Angle : " << static_cast<int>(Angle) << std::endl;
    std::cout << "Motor Measured Accel : " << get_Accel << std::endl;
    std::cout << "Motor Input Accel : " << in_Accel << std::endl;

    return 0;
}
