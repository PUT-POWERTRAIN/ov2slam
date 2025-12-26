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
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/
#pragma once

#include <memory>
#include <Eigen/Core>

#ifdef ENABLE_GPS_INIT
#include <GeographicLib/LocalCartesian.hpp>
#endif

namespace ov2slam {

class GPSConverter {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new GPSConverter object with a reference origin
     * @param lat0 Origin latitude in degrees
     * @param lon0 Origin longitude in degrees
     * @param alt0 Origin altitude in meters
     */
    GPSConverter(double lat0, double lon0, double alt0);

    /**
     * @brief Convert WGS84 coordinates to ENU (East-North-Up) local coordinates
     * @param lat Latitude in degrees
     * @param lon Longitude in degrees
     * @param alt Altitude in meters
     * @return Eigen::Vector3d ENU coordinates (x=east, y=north, z=up) in meters
     */
    Eigen::Vector3d wgs84ToENU(double lat, double lon, double alt) const;

private:
#ifdef ENABLE_GPS_INIT
    std::unique_ptr<GeographicLib::LocalCartesian> converter_;
#endif
    Eigen::Vector3d origin_;
};

} // namespace ov2slam
