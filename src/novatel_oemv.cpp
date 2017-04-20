#include "novatel_oemv.h"

GPSDriverNovAtelOEMV::GPSDriverNovAtelOEMV(GPSCallbackPtr callback, void *callback_user,
                                           struct vehicle_gps_position_s *gps_position,
                                           struct satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _gps_position(gps_position),
    _satellite_info(satellite_info),
    _lastMessage{0},
    _lastMessageSize(0),
    _lastBlock()
{

}

GPSDriverNovAtelOEMV::~GPSDriverNovAtelOEMV()
{

}

int GPSDriverNovAtelOEMV::receive(unsigned timeout)
{
    uint8_t buffer[_messageMaxSize] = {0};

    gps_abstime timeStarted = gps_absolute_time();

    while (true)
    {
        int ret = read(buffer, sizeof(buffer), timeout);

        if (ret < 0)
        {
            PX4_WARN("NovAtel: Reading problem");
            return ret;
        }
        else if (ret == 0)
        {
            // waiting

//            PX4_WARN("NovAtel: Nothing is readed");
//            return ret;
        }
        else
        {
            PX4_WARN("NovAtel: Readed %d bytes:", ret);

            parseData(buffer, ret);
//            if(ret >= 3)
//                PX4_WARN("%x %x %x ", buffer[0], buffer[1], buffer[2]);

//            PX4_WARN("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  ",
//                    static_cast<unsigned int>(buffer[0]),
//                    static_cast<unsigned int>(buffer[1]),
//                    static_cast<unsigned int>(buffer[2]),
//                    static_cast<unsigned int>(buffer[3]),
//                    static_cast<unsigned int>(buffer[4]),
//                    static_cast<unsigned int>(buffer[5]),
//                    static_cast<unsigned int>(buffer[6]),
//                    static_cast<unsigned int>(buffer[7]),
//                    static_cast<unsigned int>(buffer[8]),
//                    static_cast<unsigned int>(buffer[9]),
//                    static_cast<unsigned int>(buffer[10]),
//                    static_cast<unsigned int>(buffer[11]),
//                    static_cast<unsigned int>(buffer[12]),
//                    static_cast<unsigned int>(buffer[13]),
//                    static_cast<unsigned int>(buffer[14]),
//                    static_cast<unsigned int>(buffer[15]),
//                    static_cast<unsigned int>(buffer[16]),
//                    static_cast<unsigned int>(buffer[17]),
//                    static_cast<unsigned int>(buffer[18]),
//                    static_cast<unsigned int>(buffer[19]),
//                    static_cast<unsigned int>(buffer[20]),
//                    static_cast<unsigned int>(buffer[21]),
//                    static_cast<unsigned int>(buffer[22]),
//                    static_cast<unsigned int>(buffer[23]),
//                    static_cast<unsigned int>(buffer[24]));

            return ret;
        }

        /* abort after timeout if no useful packets received */
        if (timeStarted + timeout * 1000 < gps_absolute_time())
        {
            PX4_WARN("NovAtel: Timed out at receiving (read())");
            return -1;
        }
    }
}

int GPSDriverNovAtelOEMV::configure(unsigned &baudrate, GPSHelper::OutputMode)
{
    PX4_WARN("NovAtel: configuration");

    if(baudrate != 115200)
    {
        PX4_WARN("NovAtel: wrong baudrate: %d", baudrate);
        baudrate = 115200;
    }

    const unsigned int startBaud = 9600;
    const unsigned int waitTime = 1000;

    if(setBaudrate(startBaud) != 0) // FIXME: check all baudratees after connection lost
    {
        PX4_WARN("NovAtel: Can't set %d baudrate at PX4", startBaud);
        return -1;
    }
    // Setting normal baudrate
    changeReceiverBaudrate(baudrate, waitTime);

    if(setBaudrate(baudrate) != 0)
    {
        PX4_WARN("NovAtel: Can't set %d baudrate at PX4", baudrate);
        return -1;
    }
    // Setting normal baudrate
    changeReceiverBaudrate(baudrate, waitTime);

    // Set RX/TX mode
    char modeCom[] = "interfacemode novatel novatel on\r\n";
    if(write(modeCom, strlen(modeCom)) != strlen(modeCom))
    {
        PX4_WARN("NovAtel: Can't send RX/TX mode to OEMV");
        return -1;
    }
    if(receive(waitTime) <= 0)
    {
        PX4_WARN("NovAtel: Timeout of RX/TX mode setting (may be it normal)");
    }

    // Get position data
    char dataCom[] = "log bestposb ontime 1\r\n";
    if(write(dataCom, strlen(dataCom)) != strlen(dataCom))
    {
        PX4_WARN("NovAtel: Can't send getting position to OEMV");
        return -1;
    }
    if(receive(waitTime) <= 0)
    {
        PX4_WARN("NovAtel: Timeout of getting position (may be it normal)");
    }

    return 0;
}

void GPSDriverNovAtelOEMV::parseData(uint8_t *data, size_t size)
{
    if(size == 0 || !data)
        return;

    if(size > _messageMaxSize) // FIXME: +lastSize ?
        return;

    if(size >= 3) // FIXME: make it for block below 3 bytes
    {
        // Check header (0xAA 0x44 0x12)
        if(data[0] == 0xAA &&
           data[1] == 0x44 &&
           data[2] == 0x12)
        {
            // start to collect new message
            prepareMessage();
        }
    }

    for(size_t i = 0; i < size; ++i)
    {
        _lastMessage[_lastMessageSize] = data[i];

        ++_lastMessageSize;
    }

    // Now parsing message. GNSS data is in the little endian
    // Try to determine message size
    if(_lastMessageSize >= 10)
    {
        if(!_lastBlock.dataSize || !_lastBlock.headerSize)
        {
            _lastBlock.headerSize = _lastMessage[3];
            memcpy(&_lastBlock.dataSize, _lastMessage + 8, sizeof(_lastBlock.dataSize));

//            PX4_INFO("NovAtel: Wait message of %hhd bytes (header: %hhd)", _lastBlock.headerSize + _lastBlock.dataSize + _crcSize, _lastBlock.headerSize);
        }
    }

    // FIXME: refactor
    // check datagram end
    if(_lastBlock.headerSize && _lastBlock.dataSize)
    {
        size_t needSize = _lastBlock.headerSize + _lastBlock.dataSize + _crcSize;

        if(_lastMessageSize >= needSize)
        {
            _lastMessageSize = needSize;
            PX4_INFO("NovAtel: Got full message %d bytes", _lastMessageSize);
        }
    }
}

void GPSDriverNovAtelOEMV::changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime)
{
    char baudCommand[64];

    if(sprintf(baudCommand, "com %d N 8 1 N off\r\n", baudrate))
    {
        if(write(baudCommand, strlen(baudCommand)) != strlen(baudCommand))
        {
            PX4_WARN("NovAtel: Can't send %d baudrate to OEMV", baudrate);
            return;
        }
        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of baudrate setting (may be it normal)");
        }
    }
}
