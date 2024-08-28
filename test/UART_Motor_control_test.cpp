#include <windows.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdint>

#define DATA_LENGTH 8

class CRC16 {
public:
    static int16_t calculate(const int8_t* data, size_t start, size_t length) {
        int16_t crc = 0xFFFF;
        for (size_t i = start; i < length; ++i) {
            crc ^= data[i];
            for (int8_t j = 0; j < 8; ++j) {
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
    int8_t header = 0x3E;
    int8_t ID;
    std::vector<int8_t> data{8};
    int8_t data_length;
    int16_t crc;
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

void handleWriteError(HANDLE serialPort, std::vector<int8_t> message, DWORD bytes_written) {
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

std::vector<int8_t> Readhandler(HANDLE serialPort, std::vector<int8_t> message, DWORD total_bytes_read, DWORD bytes_read, std::vector<int8_t> target) {
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
        std::cout << "Received data: ";
        for (DWORD i = 0; i < total_bytes_read; ++i) {
            std::cout << std::hex << static_cast<int>(message[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
    else {
        handleSerialPortError("Failed to read complete message");
    }
    return message;
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

std::vector<int8_t> SerialCommu(LPCWSTR port_name, DCB dcbSerialParams, COMMTIMEOUTS timeouts, std::vector<int8_t> message) {
    std::vector<int8_t> target = {0};

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
    target = Readhandler(serialPort, message, total_bytes_read, bytes_read, target);
    CloseHandle(serialPort);
    
    return target;
}

int32_t SetAccel(int8_t function_idx) {
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
    std::vector<int8_t> Accel_message = {0};
    int32_t Accel = 0x00000000;

    std::vector<int8_t> message;
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
    int16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    // Send data to COM port
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    Accel_message = SerialCommu(port_name, dcbSerialParams, timeouts, message);
    Accel = (static_cast<uint8_t>(Accel_message[7]) << 24) | (static_cast<uint8_t>(Accel_message[6]) << 16) | (static_cast<uint8_t>(Accel_message[5]) << 8) | (static_cast<uint8_t>(Accel_message[4]));
    return Accel;
}

int32_t ReadAccel(int8_t function_idx) {
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
    std::vector<int8_t> Accel_message = {0};
    int32_t Accel = 0x00000000;

    std::vector<int8_t> message;
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
    int16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    HANDLE serialPort;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    Accel_message = SerialCommu(port_name, dcbSerialParams, timeouts, message);
    Accel = (static_cast<uint8_t>(Accel_message[7]) << 24) | (static_cast<uint8_t>(Accel_message[6]) << 16) | (static_cast<uint8_t>(Accel_message[5]) << 8) | (static_cast<uint8_t>(Accel_message[4]));
    return Accel;
}

int16_t ReadSingleAngle(void) {
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
    std::vector<int8_t> AbsAngle_message = {0};
    int16_t AbsAngle = 0;

    std::vector<int8_t> message;
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

    HANDLE serialPort;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    serialPort = CreateFileW(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    // re-calculate CRC
    int16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    AbsAngle_message = SerialCommu(port_name, dcbSerialParams, timeouts, message);
    AbsAngle = (static_cast<uint8_t>(AbsAngle_message[7]) << 8) | (static_cast<uint8_t>(AbsAngle_message[6]));
    return AbsAngle;
}

void SinglePositionCtrl(uint8_t spinDirection,uint16_t maxspeed, uint16_t angleControl){
    Serialmessage s_message;
    s_message.ID = 1;
    s_message.data[0] = 0xA6;
    s_message.data[1] = spinDirection;  // 0x00 (CW) or 0x01 (CCW)
    s_message.data[2] = static_cast<uint8_t>(maxspeed & 0x00FF);
    s_message.data[3] = static_cast<uint8_t>(maxspeed & 0xFF00);
    s_message.data[4] = static_cast<uint8_t>(angleControl & 0x00FF);
    s_message.data[5] = static_cast<uint8_t>(angleControl & 0xFF00);
    s_message.data[6] = 0x00;
    s_message.data[7] = 0x00;
    s_message.data_length = DATA_LENGTH;
    std::vector<int8_t> AbsAngle_message = {0};
    int16_t AbsAngle = 0;

    std::vector<int8_t> message;
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

    HANDLE serialPort;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    LPCWSTR port_name = L"\\\\.\\COM34";
    serialPort = CreateFileW(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    // re-calculate CRC
    int16_t crc = CRC16::calculate(message.data(), 0, message.size());
    message.push_back(crc & 0xFF);
    message.push_back((crc >> 8) & 0xFF);

    AbsAngle_message = SerialCommu(port_name, dcbSerialParams, timeouts, message);
}

int main(void) {

    int16_t Angle = ReadSingleAngle();
    int32_t get_Accel = ReadAccel(0x00);
    int32_t in_Accel = SetAccel(0x00);
    std::cout << "Motor Angle : " << static_cast<int>(Angle) << std::endl;
    std::cout << "Motor Measured Accel : " << get_Accel << std::endl;
    std::cout << "Motor Input Accel : " << in_Accel << std::endl;

    return 0;
}
