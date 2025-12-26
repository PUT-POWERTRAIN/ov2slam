/**
*    This file is part of OV²SLAM.
*
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.first at onera dot fr>      (ONERA, DTIS - IVA)
*/

#include "gps_converter.hpp"
#include <iostream>

namespace ov2slam {

GPSConverter::GPSConverter(double lat0, double lon0, double alt0)
    : origin_(lat0, lon0, alt0)
{
#ifdef ENABLE_GPS_INIT
    converter_ = std::unique_ptr<GeographicLib::LocalCartesian>(
        new GeographicLib::LocalCartesian(lat0, lon0, alt0, GeographicLib::Geocentric::WGS84())
    );
#endif
}

Eigen::Vector3d GPSConverter::wgs84ToENU(double lat, double lon, double alt) const {
#ifdef ENABLE_GPS_INIT
    if( !converter_ ) {
        std::cerr << "\n[GPSConverter] Error: Converter not initialized!\n";
        return Eigen::Vector3d::Zero();
    }

    double x, y, z;
    converter_->Forward(lat, lon, alt, x, y, z);
    return Eigen::Vector3d(x, y, z);
#else
    std::cerr << "\n[GPSConverter] Error: GPS init not enabled (ENABLE_GPS_INIT not defined)!\n";
    return Eigen::Vector3d::Zero();
#endif
}

} // namespace ov2slam
