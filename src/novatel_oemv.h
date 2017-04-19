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
    struct vehicle_gps_position_s *_gps_position;
    struct satellite_info_s *_satellite_info;

    GPSDriverNovAtelOEMV(const GPSDriverNovAtelOEMV&) = delete;
    GPSDriverNovAtelOEMV &operator=(const GPSDriverNovAtelOEMV&) = delete;
};

#endif // NOVATEL_OEMV_H
