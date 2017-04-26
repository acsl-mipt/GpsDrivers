#ifndef NOVATEL_OEMV_H
#define NOVATEL_OEMV_H

#include "gps_helper.h"

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
    enum MessagesId
    {
        Log          = 1,
        Interfacemod = 3,
        Com          = 4,
        Unlogall     = 38,

        Satvis       = 48,
        Bestpos      = 42,
        Bestvel      = 99,

        Version      = 37
    };
    enum Triggers
    {
        OnNew       = 0,
        OnChanged   = 1,
        OnTime      = 2, // Output on a time interval
        OnNext      = 3,
        Once        = 4,
        OnMark      = 5
    };
    enum Ports
    {
        NoPort      = 0x0,
        Com1All     = 0x1, // All virtual ports for COM port 1
        Com2All     = 0x2,
        Com3All     = 0x3,
        ThisPortAll = 0x6, // All virtual ports for the current port
        AllPorts    = 0x8,
        Com1        = 0x20, // COM port 1, virtual port 0
        Com2        = 0x40,
        Com3        = 0x60,
        ThisPort    = 0xC0, // Current COM port, virtual port 0
    };
    enum InterfaceModes
    {
        Novatel = 1,
        Rtcm    = 2,
        Rtca    = 3
    };

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
            headerLength(sizeof(MessageHeader)),
            messageId(0),
            messageType(0x00), // Original message, binary
            portAddress(ThisPort),
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

    struct MessageCom
    {
        uint32_t port;
        uint32_t baud;
        uint32_t parity;
        uint32_t databits;
        uint32_t stopbits;
        uint32_t handshake;
        uint32_t echo;
        uint32_t breakDetection;

        MessageCom() :
            port(ThisPortAll),
            baud(115200),
            parity(0), // no
            databits(8),
            stopbits(1),
            handshake(0), // no
            echo(0), // off
            breakDetection(1) // on - default value
        {}
        void unwrapFrom(uint8_t *data)
        {
            memcpy(this, data, sizeof(MessageCom));
        }
    } __attribute__((packed));

    struct MessageInterfacemode
    {
        uint32_t port;
        uint32_t rxtype;
        uint32_t txtype;
        uint32_t responses;

        MessageInterfacemode() :
            port(ThisPortAll), // THISPORT
            rxtype(Novatel),
            txtype(Novatel),
            responses(1) // on
        {}
        void unwrapFrom(uint8_t *data)
        {
            memcpy(this, data, sizeof(MessageInterfacemode));
        }
    } __attribute__((packed));

    struct MessageUnlogall
    {
        uint32_t port;
        uint32_t held;

        MessageUnlogall() :
            port(AllPorts),
            held(0) // Does not remove logs with the HOLD parameter
        {}
        void unwrapFrom(uint8_t *data)
        {
            memcpy(this, data, sizeof(MessageUnlogall));
        }
    } __attribute__((packed));

    struct MessageLog
    {
        uint32_t port;
        uint16_t messageId;
        uint8_t  messageType;
        uint8_t  reserved;
        uint32_t trigger;
        double   period;
        double   offset;
        uint32_t hold;

        explicit MessageLog(uint16_t id = 42) :
            port(ThisPort),
            messageId(id),
            messageType(0x00), // Original message, binary
            reserved(0),
            trigger(OnTime),
            period(0.05),
            offset(0.0),
            hold(0) // Allow log to be removed by the UNLOGALL command
        {}
        static double correctPeriod(double value);
        void unwrapFrom(uint8_t *data)
        {
            memcpy(this, data, sizeof(MessageLog));
        }
    } __attribute__((packed));

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
            elev(0.0),
            az(0.0),
            trueDop(0.0),
            appDop(0.0)
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

    static const size_t _messageMaxSize = GPS_READ_BUFFER_SIZE * 2;
    static const size_t _crcSize = 4;
    static const double _requestInterval;

    // 32-bit CRC. OEMV Family Firmware Version 3.800 Reference Manual Rev 8
    static const unsigned long crc32Polynonial = 0xEDB88320L;
    static unsigned long crc32Value(int i);
    static unsigned long calculateBlockCRC32(uint8_t *data, size_t size);
    bool checkCrc(uint8_t *data, size_t size);

private:
    int collectData(uint8_t *data, size_t size);
    int parseLastMessage();
    void prepareLastMessage();
    void cutLastMessage(size_t amount);

    void handleSatvis(uint8_t *data, size_t size);
    void handleBestpos(uint8_t *data);
    void handleBestvel(uint8_t *data);
    void handleResponse(uint8_t *data, size_t size, unsigned int commandId = 0);

    bool changeReceiverBaudrate(unsigned int baudrate, unsigned int waitTime = 1000, bool silent = false);
    bool prepareReceiver(unsigned int waitTime = 1000, bool hidden = false);
    bool requestPosition(double interval, unsigned int waitTime = 1000);
    bool requestVelocity(double interval, unsigned int waitTime = 1000);
    bool requestSatelliteInfo(double interval, unsigned int waitTime = 1000);

    template <typename M>
    int serializeMessage(uint16_t messageId, const M *body);
    template <typename M>
    int serializeMessage(uint16_t messageId);

private:
    struct vehicle_gps_position_s *_gps_position;
    struct satellite_info_s *_satellite_info;

    // Exchange buffers
    uint8_t _lastCommand[_messageMaxSize / 2];
    uint8_t _lastMessage[_messageMaxSize];
    size_t _lastMessageSize;

    // Parsed messages from receiver
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

template<typename M>
int GPSDriverNovAtelOEMV::serializeMessage(uint16_t messageId, const M *body)
{
    MessageHeader header;

    header.messageId = messageId;
    header.messageLength = sizeof(M);

    memcpy(_lastCommand, &header, sizeof(header));
    int size(sizeof(header));

    memcpy(_lastCommand + size, body, sizeof(M));
    size += sizeof(M);

    uint32_t crc = calculateBlockCRC32(_lastCommand, size);
    memcpy(_lastCommand + size, &crc, sizeof(crc));
    size += sizeof(crc);

    return size;
}

template <typename M>
int GPSDriverNovAtelOEMV::serializeMessage(uint16_t messageId)
{
    M body;
    return serializeMessage<M>(messageId, &body);
}

#endif // NOVATEL_OEMV_H
