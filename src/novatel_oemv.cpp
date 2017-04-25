#include "novatel_oemv.h"
#include "lib/geo/geo.h"
#include <algorithm>

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
    if(!_gps_position)
    {
        PX4_PANIC("NovAtel: Empty pointer for gps data!");
    }
}

GPSDriverNovAtelOEMV::~GPSDriverNovAtelOEMV()
{
    // TODO: unlogall
}

int GPSDriverNovAtelOEMV::receive(unsigned timeout)
{
    uint8_t buffer[GPS_READ_BUFFER_SIZE] = {0};

    gps_abstime timeStarted = gps_absolute_time();

    while (true)
    {
        int readed = read(buffer, sizeof(buffer), timeout);

        if (readed < 0)
        {
            PX4_WARN("NovAtel: UART reading problem");
            return readed;
        }
        else if (readed == 0)
        {
            // waiting
        }
        else
        {
            return collectData(buffer, readed);
        }

        /* abort after timeout if no useful packets received */
        if (timeStarted + timeout * 1000 < gps_absolute_time())
        {
            PX4_WARN("NovAtel: Timed out at receiving: read()");
            return -1;
        }
    }
}

int GPSDriverNovAtelOEMV::configure(unsigned &baudrate, GPSHelper::OutputMode)
{
    PX4_INFO("\nNovAtel: Reconfiguration...");

    memset(_gps_position, 0, sizeof(struct vehicle_gps_position_s));
    if(_satellite_info)
        memset(_satellite_info, 0, sizeof(struct satellite_info_s));

    if(baudrate != 115200)
    {
        PX4_WARN("NovAtel: wrong baudrate: %d", baudrate);
        baudrate = 115200;
    }

    const unsigned int startBaud = 9600;
    const unsigned int waitTime = 100; // ms

    // TODO: send only binary commands
    // TODO: check & change waitTime
    // FIXME: check receiving and return correct value
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

unsigned long GPSDriverNovAtelOEMV::calculateBlockCRC32(uint8_t *data, size_t size)
{
    unsigned long ulCRC = 0;
    while ( size-- != 0 )
    {
        unsigned long ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        unsigned long ulTemp2 = crc32Value( ((int) ulCRC ^ *data++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

int GPSDriverNovAtelOEMV::collectData(uint8_t *data, size_t size)
{
    // bit 0 set: got gps position update
    // bit 1 set: got satellite info update

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
        int parsed = parseLastMessage();
        result |= parsed;

        if(parsed > 0)
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
        int result(0xC0); // FIXME: testing // receive() > 0 but bits 0,1 are zero
        uint8_t *from = _lastMessage + _lastHeader.headerLength;
        switch (_lastHeader.messageId)
        {
            case Satvis:
                handleSatvis(from, _lastHeader.messageLength);
                result |= 0x02;
                break;
            case Bestpos:
                handleBestpos(from);
                result |= 0x01;
                break;
            case Bestvel:
                // Data was updated, but publishing only by Bestpos
                handleBestvel(from);
                break;
            case Com:
                handleResponse(from, _lastHeader.messageLength);
                break;
            default:
                break;
        }

        cutLastMessage(needSize);

//        PX4_INFO("NovAtel: Ok: Got full message %d bytes, type: %d; return: %d", needSize, _lastHeader.messageId, result); // FIXME: remove
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

    _gps_position->fix_type = 1; // Data was got
    if(_lastBestpos.posType == 16)
    {
        _gps_position->fix_type = 3;
    }
    else if(_lastBestpos.posType > 16 && _lastBestpos.posType < 19)
    {
        _gps_position->fix_type = 4;
    }
    else if(_lastBestpos.posType > 31 && _lastBestpos.posType < 35)
    {
        _gps_position->fix_type = 5;
    }
    else if(_lastBestpos.posType > 47 && _lastBestpos.posType < 51)
    {
        _gps_position->fix_type = 6;
    }

    _gps_position->timestamp = gps_absolute_time();
    _gps_position->time_utc_usec = _gps_position->timestamp;

    _gps_position->satellites_used = _lastBestpos.nSolSVs;

    _gps_position->lat = _lastBestpos.lat * 10e7;
    _gps_position->lon = _lastBestpos.lon * 10e7;
    _gps_position->alt = _lastBestpos.hgt * 10e3;

    // TODO: recalculate eph, epv
    _gps_position->eph = std::max(_lastBestpos.latSigma, _lastBestpos.lonSigma);
    _gps_position->epv = _lastBestpos.hgtSigma;

    //    if (ret > 0) {
    //		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
    //	}

    ++_rate_lat_lon;
}

void GPSDriverNovAtelOEMV::handleBestvel(uint8_t *data)
{
    if(!data)
        return;

    _lastBestvel.unwrapFrom(data);

    _gps_position->s_variance_m_s = 1.0f;
    _gps_position->vel_m_s = _lastBestvel.horSpd;

    double dir(_lastBestvel.trkGnd * static_cast<double>(M_DEG_TO_RAD_F));
    _gps_position->vel_n_m_s = _lastBestvel.horSpd * cos(dir);
    _gps_position->vel_e_m_s = _lastBestvel.horSpd * sin(dir);
    _gps_position->vel_d_m_s = -_lastBestvel.vertSpd;
    _gps_position->vel_ned_valid = true;
    _gps_position->cog_rad = _wrap_pi(dir);
}

void GPSDriverNovAtelOEMV::handleResponse(uint8_t *data, size_t size, unsigned int commandId)
{
    // It's general format for command responses
    // uint32  - Response message code
    // uint8*n - String containing the ASCII response in hex coding to match the ID above

    if(!data)
        return;

    uint32_t responseId(0);
    memcpy(&responseId, data, sizeof(responseId));

    if(responseId != 1) // TODO: add list of messages (pdf)
    {
        if(commandId)
        {
            PX4_WARN("Command #%d is failed: %d. Message: %.*s", commandId, size - sizeof(responseId), data + sizeof(responseId));
        }
        else
        {
            PX4_WARN("Command's failed. Message: %.*s", size - sizeof(responseId), data + sizeof(responseId));
        }
    }
//    int ss = 2;
//    char t[4];
//    t[0] = 0x4F;
//    t[1] = 0x4B;

//    t[2] = 0x31;

//    t[3] = 0x32;
////    PX4_INFO("Response: %.*s|    %d: ", size - sizeof(responseId), data + sizeof(responseId), size - sizeof(responseId)); // FIXME: remove
//    PX4_INFO("Response: %.*s|", ss, t); // FIXME: remove
//    printf("Response: %2s|", t);
    // if !!!
    uint8_t text[size - sizeof(responseId) + 1];
    memcpy(text, data + sizeof(responseId), size - sizeof(responseId));
    text[size - sizeof(responseId)] = '\0';
    PX4_INFO("Response: %s|", text); // FIXME: remove
}

bool GPSDriverNovAtelOEMV::changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime)
{
    MessageHeader header;
    MessageCom body;

    header.messageId = Com;
    header.messageLength = sizeof(body);

    memcpy(_lastCommand, &header, sizeof(header));
    int size(sizeof(header));

    memcpy(_lastCommand + size, &body, sizeof(body));
    size += sizeof(body);

    uint32_t crc = calculateBlockCRC32(_lastCommand, size);
    memcpy(_lastCommand + size, &crc, sizeof(crc));
    size += sizeof(crc);


//    char baudCommand[48];

//    int n = snprintf(baudCommand, sizeof(baudCommand), "com %d N 8 1 N off\r\n", baudrate);
//    if(n > 0)
//    {
        if(write(_lastCommand, size) != size)
//        if(write(baudCommand, n) != n)
        {
            PX4_ERR("NovAtel: Can't send %d baudrate to OEMV", baudrate);
            return false;
        }
PX4_INFO("NovAtel: COM - size: %d", size);
        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of baudrate setting (null parser?)");
        }
        return true;
//    }

//    return false;
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
