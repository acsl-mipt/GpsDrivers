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
    usleep(timeout * 1000);
    PX4_WARN("NovAtel: receive");
    return -1;
}

int GPSDriverNovAtelOEMV::configure(unsigned &baudrate, GPSHelper::OutputMode output_mode)
{
    PX4_WARN("NovAtel: configuration");
    return 0; // FIXME: debug
}
