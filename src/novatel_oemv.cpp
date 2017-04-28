#include "novatel_oemv.h"
#include "lib/geo/geo.h"
#include <algorithm>
#include <climits>

const double GPSDriverNovAtelOEMV::_requestInterval = 0.05;

GPSDriverNovAtelOEMV::GPSDriverNovAtelOEMV(GPSCallbackPtr callback,
                                           void *callback_user,
                                           struct vehicle_gps_position_s *gps_position,
                                           struct satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _gps_position(gps_position),
    _satellite_info(satellite_info),
    _lastCommand{0},
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
    prepareReceiver(0, true);
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
            PX4_PANIC("NovAtel: UART reading problem");
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
            PX4_WARN("NovAtel: Timed out at receiving data from OEMV: read()");
            return -1;
        }
    }
}

int GPSDriverNovAtelOEMV::configure(unsigned &baudrate, GPSHelper::OutputMode)
{
    PX4_INFO("NovAtel: OEMV reconfiguration...");

    memset(_gps_position, 0, sizeof(struct vehicle_gps_position_s));
    if(_satellite_info)
        memset(_satellite_info, 0, sizeof(struct satellite_info_s));

    if(baudrate != 115200)
    {
        PX4_WARN("NovAtel: wrong baudrate: %d", baudrate);
        baudrate = 115200;
    }

    const unsigned int startBaud = 9600;
    const unsigned int waitTime = 50; // ms

    // Initial baudrate
    if(setBaudrate(startBaud) != 0)
    {
        PX4_ERR("NovAtel: Can't set %d baudrate at PX4", startBaud);
        return -1;
    }
    if(!changeReceiverBaudrate(baudrate, waitTime * 2, true)) // because it's lower baudrate
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
    if(!sendLogCommand(Bestpos, _requestInterval, waitTime))
    {
        return -1;
    }
    if(!sendLogCommand(Bestvel, _requestInterval, waitTime))
    {
        return -1;
    }
    if(_satellite_info)
    {
        sendLogCommand(Satvis, 2.0, waitTime);
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

bool GPSDriverNovAtelOEMV::checkCrc(uint8_t *data, size_t size)
{
    uint32_t crc;

    if(!data || size <= sizeof(crc))
        return false;

    memcpy(&crc, data + size - sizeof(crc), sizeof(crc));

    return (crc == calculateBlockCRC32(data, size - sizeof(crc)));
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

        int result(0xC0); // receive() > 0 but bits 0,1 are zero

        if(checkCrc(_lastMessage, needSize))
        {
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
                case Interfacemode:
                case Unlogall:
                case Log:
                    handleResponse(from, _lastHeader.messageLength, _lastHeader.messageId);
                    break;
                default:
                    break;
            }
        }
        else
        {
            PX4_WARN("Wrong CRC, message id %d", _lastHeader.messageId);
        }

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

    unserializeMessage(&_lastBestpos, data);

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

    // OEMV sends Earth-radius when it doesn't have solution
    // This value (in mm) is bigger than int32
    if(_gps_position->alt == INT_MIN || _gps_position->alt == INT_MAX)
    {
        _gps_position->alt = 0; // NOTE: Temporary fix for huge value
    }

    _gps_position->eph = ephFromGeoSigma(_lastBestpos.lat, _lastBestpos.latSigma, _lastBestpos.lonSigma);
    _gps_position->epv = _lastBestpos.hgtSigma;

    ++_rate_count_lat_lon;
}

void GPSDriverNovAtelOEMV::handleBestvel(uint8_t *data)
{
    if(!data)
        return;

    unserializeMessage(&_lastBestvel, data);

    _gps_position->s_variance_m_s = 1.0f;
    _gps_position->vel_m_s = _lastBestvel.horSpd;

    double dir(_lastBestvel.trkGnd * static_cast<double>(M_DEG_TO_RAD_F));
    _gps_position->vel_n_m_s = _lastBestvel.horSpd * cos(dir);
    _gps_position->vel_e_m_s = _lastBestvel.horSpd * sin(dir);
    _gps_position->vel_d_m_s = -_lastBestvel.vertSpd;
    _gps_position->vel_ned_valid = true;
    _gps_position->cog_rad = _wrap_pi(dir);

    ++_rate_count_vel;
}

void GPSDriverNovAtelOEMV::handleResponse(uint8_t *data, size_t size, unsigned int commandId)
{
    if(!data)
        return;

    // It's general format for command responses
    // uint32  - Response message code
    // uint8*n - String containing the ASCII response in hex coding to match the ID above

    uint32_t responseId(0);
    memcpy(&responseId, data, sizeof(responseId));

    if(responseId != ResponseOk)
    {
        // Pixhawk/Nuttx doesn't understand "%.*s"-format. And Pixhawk has very tiny memory stack...
        // So we use heap memory and add null-terminator.
        size_t textSize = size - sizeof(responseId);
        uint8_t *text = new uint8_t[textSize + 1];
        memcpy(text, data + sizeof(responseId), textSize);
        text[textSize] = '\0';

        if(commandId)
        {
            PX4_WARN("NovAtel: %s command (code %d) is failed, error %d. Message: %s", commandName(commandId), commandId, responseId, text);
        }
        else
        {
            PX4_WARN("NovAtel: Command's failed, error %d. Message: %s", responseId, text);
        }

        delete[] text;
    }
}

bool GPSDriverNovAtelOEMV::changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime, bool silent)
{
    int size = serializeMessage<MessageCom>(Com);

    if(write(_lastCommand, size) != size)
    {
        if(!silent)
            PX4_ERR("NovAtel: Can't send %d baudrate to OEMV", baudrate);

        return false;
    }

    if(receive(waitTime) <= 0)
    {
        if(!silent)
        {
            PX4_WARN("NovAtel: Timeout of baudrate setting");
            return false;
        }
    }
    return true;
}

bool GPSDriverNovAtelOEMV::prepareReceiver(unsigned int waitTime, bool hidden)
{
    bool sendProblem(false);

    int size(0);

    size = serializeMessage<MessageInterfacemode>(Interfacemode);
    sendProblem |= (write(_lastCommand, size) != size);

    size = serializeMessage<MessageUnlogall>(Unlogall);
    sendProblem |= (write(_lastCommand, size) != size);

    if(!hidden)
    {
        if(sendProblem)
        {
            PX4_ERR("NovAtel: Can't send preparation commands to OEMV");
            return false;
        }

        if(receive(waitTime) <= 0)
        {
            PX4_WARN("NovAtel: Timeout of preparation commands setting");
            return false;
        }
    }

    return true;
}

bool GPSDriverNovAtelOEMV::sendLogCommand(GPSDriverNovAtelOEMV::MessagesId command, double interval, unsigned int waitTime)
{
    MessageLog message;
    message.messageId = command;
    message.period = MessageLog::correctPeriod(interval);

    int size = serializeMessage(Log, &message);

    if(write(_lastCommand, size) != size)
    {
        PX4_ERR("NovAtel: Can't send %s command to OEMV", commandName(command));
        return false;
    }

    if(receive(waitTime) <= 0)
    {
        PX4_WARN("NovAtel: Timeout of receiving %s command from OEMV", commandName(command));
        return false;
    }
    return true;
}

const char *GPSDriverNovAtelOEMV::commandName(unsigned int id)
{
    // Upper case like in Reference
    switch (id)
    {
        case Log:           return "LOG";
        case Interfacemode: return "INTERFACEMODE";
        case Com:           return "COM";
        case Unlogall:      return "UNLOGALL";
        case Satvis:        return "SATVIS";
        case Bestpos:       return "BESTPOS";
        case Bestvel:       return "BESTVEL";
        case Version:       return "VERSION";
        default:            return "UNKNOWN";
    }
}

float GPSDriverNovAtelOEMV::ephFromGeoSigma(double latitude,
                                            double latitudeSigma, double longtitudeSigma)
{
    double refLat = latitude + 0.5 * latitudeSigma;
    double nm = latitudeSigma * 333400.0 / 3.0;
    double em = longtitudeSigma * 1001879.0 * std::cos(refLat * M_PI / 180.0) / 9.0;

    return static_cast<float>(std::sqrt(em * em + nm * nm));
}

double GPSDriverNovAtelOEMV::MessageLog::correctPeriod(double value)
{
    if(value <= 0.05)
        return 0.05;
    else if(value <= 0.1)
        return 0.1;
    else if(value <= 0.2)
        return 0.2;
    else if(value <= 0.25)
        return 0.25;
    else if(value <= 0.5)
        return 0.5;
    else
        return std::floor(value);
}
