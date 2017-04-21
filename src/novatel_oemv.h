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
    // Inner types and defines
    struct DataBlock
    {
        bool parsed;
        uint8_t  headerSize;
        uint16_t messageId;
        uint16_t dataSize;


        DataBlock() :
            parsed(false),
            headerSize(0),
            messageId(0),
            dataSize(0)
        {}
        bool isParsed() const {return parsed;}
    };
    static const size_t _headerMinimumSize = 3 + 25;
    static const size_t _messageMaxSize = GPS_READ_BUFFER_SIZE * 2;
    static const size_t _crcSize = 4;
    static const double _requestInterval;

private:
    void collectData(uint8_t *data, size_t size);
    bool parseLastMessage();
    void prepareLastMessage();
    void cutLastMessage(size_t amount);

    bool changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime = 1000);
    bool prepareReceiver(unsigned int waitTime = 1000);
    bool requestPosition(double interval, unsigned int waitTime = 1000);
    bool requestVelocity(double interval, unsigned int waitTime = 1000);
    bool requestSatelliteInfo(double interval, unsigned int waitTime = 1000);

private:
    struct vehicle_gps_position_s *_gps_position;
    struct satellite_info_s *_satellite_info;

    uint8_t _lastMessage[_messageMaxSize];
    size_t _lastMessageSize;

    DataBlock _lastBlock;

    GPSDriverNovAtelOEMV(const GPSDriverNovAtelOEMV&) = delete;
    GPSDriverNovAtelOEMV &operator=(const GPSDriverNovAtelOEMV&) = delete;
};

inline void GPSDriverNovAtelOEMV::prepareLastMessage()
{
    for(size_t i = 0; i < _messageMaxSize; ++i)
    {
        _lastMessage[i] = 0;
    }
    _lastMessageSize = 0;
}

#endif // NOVATEL_OEMV_H
