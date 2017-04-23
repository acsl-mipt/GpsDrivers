#ifndef NOVATEL_OEMV_H
#define NOVATEL_OEMV_H

#include "gps_helper.h"
#include "../../definitions.h"

class GPSDriverNovAtelOEMV : public GPSHelper
{
public:
    GPSDriverNovAtelOEMV(GPSCallbackPtr callback,
                         void *callback_user,
                         struct vehicle_gps_position_s *gps_position,
                         struct satellite_info_s *satellite_info);
    ~GPSDriverNovAtelOEMV();
    int receive(unsigned timeout);
    int configure(unsigned &baudrate, OutputMode output_mode);

private:
    // Inner types and defines
    struct MessageHeader
    {
        uint8_t  sync[3];
        uint8_t  headerLength;
        uint16_t messageId;
        uint8_t  messageType;
        uint8_t  portAddress;
        uint16_t messageLength;
        uint16_t sequence;
        uint8_t  idleTime;
        uint8_t  timeStatus;
        uint16_t week;
        uint32_t ms;
        uint32_t receiverStatus;
        uint16_t reserved;
        uint16_t receiverSWVersion;

        MessageHeader() :
            sync{0xAA, 0x44, 0x12},
            headerLength(0),
            messageId(0),
            messageType(0x00), // Original message, binary
            portAddress(0xC0), // This port
            messageLength(0),
            sequence(0),
            idleTime(0),
            timeStatus(0),
            week(0),
            ms(0),
            receiverStatus(0),
            reserved(0),
            receiverSWVersion(0)
        {}

        uint8_t syncByte(size_t index) const
        {
            return (index < 3) ? sync[index] : 0;
        }
    } __attribute__((packed));

    enum MessagesId
    {
        Satvis  = 48,
        Bestpos = 49,
        Bestvel = 99
    };

    struct MessageLogSatvisPrefix
    {
        uint32_t satVis;
        uint32_t compAlm;
        uint32_t nSat;

        MessageLogSatvisPrefix() :
            satVis(0),
            compAlm(0),
            nSat(0)
        {}
    } __attribute__((packed));

    struct MessageLogSatvisBlock
    {
        uint16_t prnSlot;
        uint16_t glofreq;
        uint32_t health;
        double   elev;
        double   az;
        double   trueDop;
        double   appDop;

        MessageLogSatvisBlock() :
            prnSlot(0),
            glofreq(0),
            health(0),
            elev(0.0f),
            az(0.0f),
            trueDop(0.0f),
            appDop(0.0f)
        {}
    } __attribute__((packed));

    struct MessageLogBestpos
    {
        uint32_t solStat;
        uint32_t posType;
        double   lat;
        double   lon;
        double   hgt;
        float    undulation;
        uint32_t datumIdN;
        float    latSigma;
        float    lonSigma;
        float    hgtSigma;
        uint8_t  stnId[4];
        float    diffAge;
        float    solAge;
        uint8_t  nSVs;
        uint8_t  nSolSVs;
        uint8_t  nggL1;
        uint8_t  nggL1L2;
        uint8_t  reserved0;
        uint8_t  extSolStat;
        uint8_t  reserved1;
        uint8_t  sigMask;

        MessageLogBestpos()
        {
            memset(this, 0, sizeof(MessageLogBestpos));
        }
        void unwrapFrom(uint8_t *data)
        {
            memcpy(this, data, sizeof(MessageLogBestpos));
        }
    } __attribute__((packed));

    struct MessageLogBestvel
    {
        uint32_t solStatus;
        uint32_t velType;
        float    latency;
        float    age;
        double   horSpd;
        double   trkGnd;
        double   vertSpd;
        float    reserved;

        MessageLogBestvel()
        {
            memset(this, 0, sizeof(MessageLogBestvel));
        }
        void unwrapFrom(uint8_t *data)
        {
            memcpy(this, data, sizeof(MessageLogBestvel));
        }
    } __attribute__((packed));

    struct DataBlock // FIXME: remove it (?)
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

    static const size_t _messageMaxSize = GPS_READ_BUFFER_SIZE * 2;
    static const size_t _crcSize = 4;
    static const double _requestInterval;

    // 32-bit CRC. OEMV Family Firmware Version 3.800 Reference Manual Rev 8
    static const unsigned long crc32Polynonial = 0xEDB88320L;
    static unsigned long crc32Value(int i);
    static unsigned long calculateBlockCRC32(
            unsigned long ulCount,    /* Number of bytes in the data block */
            unsigned char *ucBuffer ); /* Data block */

private:
    int collectData(uint8_t *data, size_t size);
    int parseLastMessage();
    void prepareLastMessage();
    void cutLastMessage(size_t amount);

    void handleSatvis(uint8_t *data, size_t size);
    void handleBestpos(uint8_t *data/*, size_t size = sizeof(MessageLogBestpos)*/);
    void handleBestvel(uint8_t *data/*, size_t size = sizeof(MessageLogBestvel)*/);

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

    // From receiver message
    MessageHeader     _lastHeader;
    MessageLogBestpos _lastBestpos;
    MessageLogBestvel _lastBestvel;


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
