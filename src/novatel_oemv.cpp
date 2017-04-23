#include "novatel_oemv.h"

const double GPSDriverNovAtelOEMV::_requestInterval = 0.05f;

GPSDriverNovAtelOEMV::GPSDriverNovAtelOEMV(GPSCallbackPtr callback,
                                           void *callback_user,
                                           struct vehicle_gps_position_s *gps_position,
                                           struct satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _gps_position(gps_position),
    _satellite_info(satellite_info),
    _lastMessage{0},
    _lastMessageSize(0),
    _lastHeader(),
    _lastBestpos(),
    _lastBestvel()
{
    if(!_gps_position || !_satellite_info)
    {
        PX4_PANIC("NovAtel: Empty pointer for gps data!");
    }
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
            // bit 0 set: got gps position update
            // bit 1 set: got satellite info update

            return collectData(buffer, ret);
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
    const unsigned int waitTime = 100; // ms

    // TODO: send only binary commands
    // FIXME: check & change waitTime
    // Initial baudrate
    if(setBaudrate(startBaud) != 0)
    {
        PX4_ERR("NovAtel: Can't set %d baudrate at PX4", startBaud);
        return -1;
    }
    if(!changeReceiverBaudrate(baudrate, waitTime * 4)) // because it's lower baudrate
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

unsigned long GPSDriverNovAtelOEMV::crc32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ crc32Polynonial;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

unsigned long GPSDriverNovAtelOEMV::calculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer)
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = crc32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

int GPSDriverNovAtelOEMV::collectData(uint8_t *data, size_t size)
{
    if(!size || !data)
        return 0;

    // start new collecting
    if(size + _lastMessageSize > _messageMaxSize)
    {
        prepareLastMessage();
    }

    // Very big message - ignore it
    if(size > _messageMaxSize)
        return 0;

    for(size_t i = 0; i < size; ++i)
    {
        _lastMessage[_lastMessageSize] = data[i];

        ++_lastMessageSize;
    }

    // Try to find binary data header
    int result(0);
    while(true)
    {
        result |= parseLastMessage();
        if(result > 0)
        {
            // Buffer has data after parsing
            if(_lastMessageSize)
            {
                continue;
            }
        }

        break;
    }

    return result;
}

int GPSDriverNovAtelOEMV::parseLastMessage()
{
    // Parse binary
    // GNSS data is in the little endian
    if(_lastMessageSize < sizeof(MessageHeader))
    {
        // nothing to parse
        return 0;
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
            return 0;
        }
    }

    // check datagram end
    uint16_t dataSize(0);
    memcpy(&dataSize, _lastMessage + 8, sizeof(dataSize));

    size_t needSize = _lastMessage[3] + dataSize + _crcSize;

    if(_lastMessageSize >= needSize &&
       _lastMessage[3] == sizeof(MessageHeader))
    {
        // Start parsing
        memcpy(&_lastHeader, _lastMessage, sizeof(_lastHeader));

        // TODO: check CRC
        int result(0);
        switch (_lastHeader.messageId)
        {
            case Satvis:
                handleSatvis(_lastMessage + _lastHeader.headerLength, _lastHeader.messageLength);
                result |= 0x02;
                break;
            case Bestpos:
                handleBestpos(_lastMessage + _lastHeader.headerLength);
                result |= 0x01;
                break;
            case Bestvel:
                handleBestvel(_lastMessage + _lastHeader.headerLength);
                result |= 0x01;
                break;
            default:
                break;
        }

        PX4_INFO("NovAtel: Ok: Got full message %d bytes, type: %d", needSize, _lastHeader.messageId);

        cutLastMessage(needSize);

        return result;
    }

    return 0;
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

void GPSDriverNovAtelOEMV::handleSatvis(uint8_t *, size_t )
{
    // TODO: realize it (if it will be usefull)
}

void GPSDriverNovAtelOEMV::handleBestpos(uint8_t *data)
{
    if(!data)
        return;

    _lastBestpos.unwrapFrom(data);

    // FIXME: determine fix_type (ask Korotkov)

    _gps_position->satellites_used = _lastBestpos.nSolSVs;

    _gps_position->lat = _lastBestpos.lat * 10e7;
    _gps_position->lon = _lastBestpos.lon * 10e7;
    _gps_position->alt = _lastBestpos.hgt * 10e3;

    _gps_position->timestamp = gps_absolute_time();

    ++_rate_lat_lon;


//    if (ret > 0) {
//		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
//	}
}

void GPSDriverNovAtelOEMV::handleBestvel(uint8_t *data)
{
    // FIXME: realization
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
            PX4_WARN("NovAtel: Timeout of baudrate setting (null parser?)");
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
        PX4_WARN("NovAtel: Timeout of preparation commands setting (null parser?)");
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
            PX4_WARN("NovAtel: Timeout of getting position (null parser?)");
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
            PX4_WARN("NovAtel: Timeout of getting velocity (null parser?)");
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
            PX4_WARN("NovAtel: Timeout of getting satellites (null parser?)");
        }

        return true;
    }

    return false;
}
