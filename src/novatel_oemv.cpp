#include "novatel_oemv.h"

GPSDriverNovAtelOEMV::GPSDriverNovAtelOEMV(GPSCallbackPtr callback, void *callback_user,
                                           struct vehicle_gps_position_s *gps_position,
                                           struct satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _gps_position(gps_position),
    _satellite_info(satellite_info)
{

}

GPSDriverNovAtelOEMV::~GPSDriverNovAtelOEMV()
{

}

int GPSDriverNovAtelOEMV::receive(unsigned timeout)
{
    uint8_t buffer[512];

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

            for(unsigned int i = 0; i < ret; ++ret)
            {
                printf("%X ", buffer[i]);
            }
            printf("\n");

//            PX4_WARN("%s", text);
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

    if(setBaudrate(startBaud) != 0)
    {
        PX4_WARN("NovAtel: Can't set %d baudrate at PX4", startBaud);
        return -1;
    }

    // Setting normal baudrate
    char baudCom[] = "com 115200 N 8 1 N off\r\n";
    if(write(baudCom, strlen(baudCom)) != strlen(baudCom))
    {
        PX4_WARN("NovAtel: Can't send %d baudrate to OEMV", baudrate);
        return -1;
    }
    if(receive(waitTime) <= 0)
    {
        PX4_WARN("NovAtel: Timeout of baudrate setting (may be it normal)");
    }

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
