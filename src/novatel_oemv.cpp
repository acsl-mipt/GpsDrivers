#include "novatel_oemv.h"

const double GPSDriverNovAtelOEMV::_requestInterval = 0.05f;

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
    uint8_t buffer[GPS_READ_BUFFER_SIZE] = {0};

    gps_abstime timeStarted = gps_absolute_time();

    while (true)
    {
        int ret = read(buffer, sizeof(buffer), timeout);

        if (ret < 0)
        {
            PX4_WARN("NovAtel: UART reading problem");
            return ret;
        }
        else if (ret == 0)
        {
            // waiting
        }
        else
        {
            collectData(buffer, ret);

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
    PX4_INFO("\n");
    PX4_INFO("NovAtel: Reconfiguration...");

    if(baudrate != 115200)
    {
        PX4_WARN("NovAtel: wrong baudrate: %d", baudrate);
        baudrate = 115200;
    }

    const unsigned int startBaud = 9600;
    const unsigned int waitTime = 500;

    // Initial baudrate
    if(setBaudrate(startBaud) != 0)
    {
        PX4_ERR("NovAtel: Can't set %d baudrate at PX4", startBaud);
        return -1;
    }
    if(!changeReceiverBaudrate(baudrate, waitTime))
    {
        return -1;
    }

    // Work baudrate
    if(setBaudrate(baudrate) != 0)
    {
        PX4_ERR("NovAtel: Can't set %d baudrate at PX4", baudrate);
        return -1;
    }
    // Additional settings sending (for case if wrong settings were at receiver from last time)
    if(!changeReceiverBaudrate(baudrate, waitTime))
    {
        return -1;
    }

    if(!prepareReceiver(waitTime))
    {
        return -1;
    }

    // Get work data
//    if(!requestSatelliteInfo(2.0f, 150))
//    {
//        return -1;
//    }
    if(!requestPosition(_requestInterval, waitTime))
    {
        return -1;
    }
    if(!requestVelocity(_requestInterval, waitTime))
    {
        return -1;
    }

    return 0;
}

void GPSDriverNovAtelOEMV::collectData(uint8_t *data, size_t size)
{
    if(!size || !data)
        return;

    // start new collecting
    if(size + _lastMessageSize > _messageMaxSize)
    {
        prepareLastMessage();
    }

    // Very big message - ignore it
    if(size > _messageMaxSize)
        return;

    for(size_t i = 0; i < size; ++i)
    {
        _lastMessage[_lastMessageSize] = data[i];

        ++_lastMessageSize;
    }

    // Try to find binary data header
    while(true)
    {
        if(parseLastMessage())
        {
            // Buffer has data after parsing
            if(_lastMessageSize)
            {
                continue;
            }
        }

        break;
    }
}

bool GPSDriverNovAtelOEMV::parseLastMessage()
{
    // Parse binary
    // GNSS data is in the little endian
    if(_lastMessageSize < _headerMinimumSize)
    {
        // nothing to parse
        return false;
    }

    if(_lastMessage[0] != 0xAA ||
       _lastMessage[1] != 0x44 ||
       _lastMessage[2] != 0x12)
    {
        // Finding next datagram in message buffer
        size_t newStart(_lastMessageSize);

        for(size_t i = 0; i < _lastMessageSize; ++i)
        {
            // 1st header byte is not at start of message
            if(_lastMessage[i] == 0xAA && i > 0)
            {
                // have enough bytes
                if(i + 2 < _lastMessageSize)
                {
                    if(_lastMessage[i + 1] == 0x44 &&
                       _lastMessage[i + 2] == 0x12)
                    {
                        newStart = i;
                        break;
                    }
                }
            }
        }

        cutLastMessage(newStart);

        // Nothing is found
        if(newStart == _lastMessageSize)
        {
            // Wrong data. Will wait next...
            return false;
        }
    }


    // check datagram end
    uint16_t dataSize(0);
    memcpy(&dataSize, _lastMessage + 8, sizeof(dataSize));

    size_t needSize = _lastMessage[3] + dataSize + _crcSize;
    // "The header length is variable as fields may be appended in the future."
    if(_lastMessageSize >= needSize &&
       _lastMessage[3] >= _headerMinimumSize)
    {
        // Start parsing
        _lastBlock = DataBlock();

        _lastBlock.headerSize = _lastMessage[3];
        _lastBlock.dataSize = dataSize;

        memcpy(&_lastBlock.messageId, _lastMessage + 4, sizeof(_lastBlock.messageId));

        PX4_INFO("NovAtel: Ok: Got full message %d bytes, type: %d", needSize, _lastBlock.messageId);

        // FIXME: inner message parsing

        cutLastMessage(needSize);

        return true;
    }

    return false;
}

void GPSDriverNovAtelOEMV::cutLastMessage(size_t amount)
{
    // Cut bytes from the beginning of message
    if(amount)
    {
        if(amount < _lastMessageSize)
        {
            _lastMessageSize -= amount;
            for(size_t i = 0; i < _lastMessageSize; ++i)
            {
                _lastMessage[i] = _lastMessage[i + amount];
            }
        }
        else
        {
            prepareLastMessage();
        }
    }
}

bool GPSDriverNovAtelOEMV::changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime)
{
    char baudCommand[48];

    int n = snprintf(baudCommand, sizeof(baudCommand), "com %d N 8 1 N off\r\n", baudrate);
    if(n > 0)
    {
        if(write(baudCommand, n) != n)
        {
            PX4_ERR("NovAtel: Can't send %d baudrate to OEMV", baudrate);
            return false;
        }

        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of baudrate setting (may be it's normal)");
        }
        return true;
    }

    return false;
}

bool GPSDriverNovAtelOEMV::prepareReceiver(unsigned int waitTime)
{
    // Set RX/TX mode
    // stop all previous logs
    char modeCommmand[] = "interfacemode novatel novatel on\r\n"
                          "unlogall\r\n";
    if(write(modeCommmand, strlen(modeCommmand)) != strlen(modeCommmand))
    {
        PX4_ERR("NovAtel: Can't send preparation commands to OEMV");
        return false;
    }

    if(receive(waitTime) <= 0)
    {
        PX4_WARN("NovAtel: Timeout of preparation commands setting (may be it's normal)");
    }

    return true;
}

bool GPSDriverNovAtelOEMV::requestPosition(double interval, unsigned int waitTime)
{
    char posCommand[48];

    int n = snprintf(posCommand, sizeof(posCommand), "log bestposb ontime %4.2f\r\n", interval);
    if(n > 0)
    {
        if(write(posCommand, n) != n)
        {
            PX4_ERR("NovAtel: Can't send position request to OEMV");
            return false;
        }
        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of getting position (may be it's normal)");
        }

        return true;
    }

    return false;
}

bool GPSDriverNovAtelOEMV::requestVelocity(double interval, unsigned int waitTime)
{
    char velCommand[48];

    int n = snprintf(velCommand, sizeof(velCommand), "log bestvelb ontime %4.2f\r\n", interval);
    if(n > 0)
    {
        if(write(velCommand, n) != n)
        {
            PX4_ERR("NovAtel: Can't send velocity request to OEMV");
            return false;
        }
        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of getting velocity (may be it's normal)");
        }

        return true;
    }

    return false;
}

bool GPSDriverNovAtelOEMV::requestSatelliteInfo(double interval, unsigned int waitTime)
{
    char velCommand[48];

    int n = snprintf(velCommand, sizeof(velCommand), "log satvisb ontime %f\r\n", interval);
    if(n > 0)
    {
        if(write(velCommand, n) != n)
        {
            PX4_ERR("NovAtel: Can't send satellites request to OEMV");
            return false;
        }
        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of getting satellites (may be it's normal)");
        }

        return true;
    }

    return false;
}
