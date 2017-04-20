#ifndef NOVATEL_OEMV_H
#define NOVATEL_OEMV_H

#include "gps_helper.h"
#include "../../definitions.h"

class GPSDriverNovAtelOEMV : public GPSHelper
{
public:
    GPSDriverNovAtelOEMV(GPSCallbackPtr callback, void *callback_user,
                         struct vehicle_gps_position_s *gps_position,
                         struct satellite_info_s *satellite_info);
    ~GPSDriverNovAtelOEMV();
    int receive(unsigned timeout);
    int configure(unsigned &baudrate, OutputMode output_mode);

private:
    void parseData(uint8_t *data, size_t size);
    void prepareMessage();
    void changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime = 1000);

private:
    struct DataBlock
    {
        uint8_t  headerSize;
        uint16_t dataSize;

        DataBlock() :
            headerSize(0),
            dataSize(0)
        {}
    };
    static const size_t _messageMaxSize = 256;
    static const size_t _crcSize = 4;

    struct vehicle_gps_position_s *_gps_position;
    struct satellite_info_s *_satellite_info;

    uint8_t _lastMessage[_messageMaxSize];
    size_t _lastMessageSize;

    DataBlock _lastBlock;

    GPSDriverNovAtelOEMV(const GPSDriverNovAtelOEMV&) = delete;
    GPSDriverNovAtelOEMV &operator=(const GPSDriverNovAtelOEMV&) = delete;
};

inline void GPSDriverNovAtelOEMV::prepareMessage()
{
    for(size_t i = 0; i < _messageMaxSize; ++i)
    {
        _lastMessage[i] = 0;
    }
    _lastMessageSize = 0;

    _lastBlock = DataBlock();
}

#endif // NOVATEL_OEMV_H
