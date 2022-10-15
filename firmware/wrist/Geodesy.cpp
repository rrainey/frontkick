/*
 * This file is part of the Kick distribution (https://github.com/rrainey/frontkick
 * Copyright (c) 2022 Riley Rainey
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include "Geodesy.h"

#include <stdio.h>
#include <math.h>
#include <cctype>
#include <cstring>

using namespace std;
using namespace Geodesy;

const double EPSILON = 0.000000000001;
const double HALFPI = 1.57079632679489661923;

Matrix4d identity_matrix (Matrix4d::Identity);
Matrix4d zero_matrix (Matrix4d::Zero);

// Transform a Vector3d
// "m" is a 3x3 transformation matrix augmented with offset information

Vector3d Matrix4d::operator * (const Vector3d & v)
const
{
    return
        v.
        Transform (*this, Augmented);
}

Vector3d Vector3d::Transform (const Matrix4d & m,
                              AugmentationType t /*=Augmented*/ ) const
{
    if (t == Augmented)
    {
        return Vector3d (x * m.mat (0, 0) + y * m.mat (0, 1) +
            z * m.mat (0, 2) + m.mat (0, 3), x * m.mat (1, 0) +
            y * m.mat (1, 1) + z * m.mat (1, 2) + m.mat (1, 3),
            x * m.mat (2, 0) + y * m.mat (2, 1) + z * m.mat (2, 2) + m.mat (2, 3)
            );
    }
    else
    {
        return Vector3d (x * m.mat (0, 0) + y * m.mat (0, 1) +
            z * m.mat (0, 2), x * m.mat (1, 0) + y * m.mat (1,
            1)
            + z * m.mat (1, 2), x * m.mat (2,
            0) + y * m.mat (2,
            1)
            + z * m.mat (2, 2)
            );
    }
}

Matrix4d::Matrix4d (const double latitude, const double longitude)
{
    double s2, s3, c2, c3;

    s2 = sin (latitude);
    s3 = sin (longitude);
    c2 = cos (latitude);
    c3 = cos (longitude);

    m[0][0] = c2 * c3;
    m[0][1] = c2 * s3;
    m[0][2] = -s2;
    m[1][0] = -s3;
    m[1][1] = c3;
    m[1][2] = 0.0;
    m[2][0] = s2 * c3;
    m[2][1] = s2 * s3;
    m[2][2] = c2;
    m[0][3] = m[1][3] = m[2][3] = m[3][0] = m[3][1] = m[3][2] = 0.0;
    m[3][3] = 1.0;
}

// multiplication of two matrices

Matrix4d Matrix4d::operator * (const Matrix4d & b)
const
{
    int
        i,
        j,
        k;
    double
        x;
    Matrix4d
        result;

    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            x = 0.0;
            for (k = 0; k < 4; ++k)
            {
                x += m[k][i] * b.m[j][k];
            }
            result.m[j][i] = x;
        }
    }
    return result;
}

void Matrix4d::Translate (const Vector3d & x)
{
    mat (0, 3) =
        x.x * mat (0, 0) + x.y * mat (0, 1) + x.z * mat (0, 2) + mat (0, 3);
    mat (1, 3) =
        x.x * mat (1, 0) + x.y * mat (1, 1) + x.z * mat (1, 2) + mat (1, 3);
    mat (2, 3) =
        x.x * mat (2, 0) + x.y * mat (2, 1) + x.z * mat (2, 2) + mat (2, 3);
    mat (3, 3) =
        x.x * mat (3, 0) + x.y * mat (3, 1) + x.z * mat (3, 2) + mat (3, 3);
}
void Matrix4d::Translate (double x, double y, double z)
{
  mat (0, 3) =
      x * mat (0, 0) + y * mat (0, 1) + z * mat (0, 2) + mat (0, 3);
  mat (1, 3) =
      x * mat (1, 0) + y * mat (1, 1) + z * mat (1, 2) + mat (1, 3);
  mat (2, 3) =
      x * mat (2, 0) + y * mat (2, 1) + z * mat (2, 2) + mat (2, 3);
  mat (3, 3) =
      x * mat (3, 0) + y * mat (3, 1) + z * mat (3, 2) + mat (3, 3);
}

void Matrix4d::Scale (double x, double y, double z)
{
    mat (0, 0) *= x;
    mat (0, 1) *= y;
    mat (0, 2) *= z;
    mat (1, 0) *= x;
    mat (1, 1) *= y;
    mat (1, 2) *= z;
    mat (2, 0) *= x;
    mat (2, 1) *= y;
    mat (2, 2) *= z;
    mat (3, 0) *= x;
    mat (3, 1) *= y;
    mat (3, 2) *= z;
}

void Matrix4d::Matrix4dToEuler (double *dPhi, double *dTheta, double *dPsi) const
{
    *dPhi = atan2 (mat1 (2, 3), mat1 (3, 3));
    *dTheta = -asin (mat1 (1, 3));
    *dPsi = atan2 (mat1 (1, 2), mat1 (1, 1));
    if (*dPsi < 0.0)
    {
        *dPsi += 2.0 * M_PI;
    }

}
// print Matrix4d
std::ostream & operator << (std::ostream & o, const Matrix4d & m)
{
    int i1, i, j, k, n, max = 0;
    char str[16][128];
    double x;

    // First, determine the length of the longest printed Matrix4d element
    n = 0;
    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            x = m.m[j][i];
            sprintf (&str[n][0], "%g", x);
            k = strlen (&str[n][0]);
            if (k > max)
            {
                max = k;
            }
            ++n;
        }
    }

    // Now print them ...

    n = 0;
    for (i = 0; i < 4; ++i)
    {
        o << "|  ";
        for (j = 0; j < 4; ++j)
        {
            n = i * 4 + j;
            k = max - strlen (&str[n][0]);
            for (i1 = 0; i1 < k; ++i1)
            {
                o << " ";
            }
            o << &str[n][0];
            o << "  ";
            ++n;
        }
        o << "|\n";
    }

    return o;
}


#ifdef notdef
// cast Matrix4d to quaternion

Matrix4d::operator Quaternion () const
{
    quaternion q;
    double tr, s, w, x, y, z, qi;
    int i, j, k;

    tr = mat (0, 0) + mat (1, 1) + mat (2, 2);
    if (tr > 0.0)
    {
        s = sqrt (tr + 1.0);
        w = 0.5 * s;
        s = 0.5 / s;

        x = (mat (1, 2) - mat (2, 1)) * s;
        y = (mat (2, 0) - mat (0, 2)) * s;
        z = (mat (0, 1) - mat (1, 0)) * s;
        q = Quaternion (w, x, y, z);
    }
    else
    {
        i = 0;
        if (mat (1, 1) > mat (0, 0))
            i = 1;
        if (mat (2, 2) > mat (i, i))
            i = 2;
        if ((j = i + 1) > 2)
            j = 0;
        if ((k = j + 1) > 2)
            k = 0;

        s = sqrt (mat (i, i) - (mat (j, j) + mat (k, k)) + 1.0);
        qi = s * 0.5;
        s = 0.5 / s;
        switch (i)
        {
            case 0:
                q = Quaternion ((mat (j, k) - mat (k, j)) * s,
                    qi,
                    (mat (i, j) - mat (j, i)) * s,
                    (mat (i, k) - mat (k, i)) * s);
                break;
            case 1:
                q = Quaternion ((mat (j, k) - mat (k, j)) * s,
                    (mat (i, k) - mat (k, i)) * s,
                    qi, (mat (i, j) - mat (j, i)) * s);
                break;
            case 2:
                q = Quaternion ((mat (j, k) - mat (k, j)) * s,
                    (mat (i, j) - mat (j, i)) * s,
                    (mat (i, k) - mat (k, i)) * s, qi);
                break;
        }
    }
    return q;
}


// cast quaternion to Matrix4d

Quaternion::operator Matrix4d () const
{
    Matrix4d mat (Matrix4d::Identity);
    Vector3d vs, w, x;
    double s0, yy, yz, zz;

    s0 = 2.0 / (s * s + v.x * v.x + v.y * v.y + v.z * v.z);

    vs = v * s0;
    w = s * vs;
    x = v.x * vs;
    yy = v.y * vs.y;
    yz = v.y * vs.z;
    zz = v.z * vs.z;

    mat.mat (0, 0) = 1.0 - (yy + zz);
    mat.mat (0, 1) = x.y + w.z;
    mat.mat (0, 2) = x.z - w.y;

    mat.mat (1, 0) = x.y - w.z;
    mat.mat (1, 1) = 1.0 - (x.x + zz);
    mat.mat (1, 2) = yz + w.x;

    mat.mat (2, 0) = x.z + w.y;
    mat.mat (2, 1) = yz - w.x;
    mat.mat (2, 2) = 1.0 - (x.x + yy);

    return mat;
}


Quaternion
Quaternion::Interpolate (const Quaternion & p, const Quaternion & q,
const double t)
{
    double sin_omega, cos_omega, omega, sp, sq;
    Quaternion r;

    cos_omega = p.s * q.s + p.v ^ q.v;

    if ((1.0 + cos_omega) > EPSILON)
    {
        if ((1.0 - cos_omega) > EPSILON)
        {
            omega = acos (cos_omega);
            sin_omega = sin (omega);
            sp = sin ((1.0 - t) * omega) / sin_omega;
            sq = sin (t * omega) / sin_omega;
        }
        else
        {
            sp = 1.0 - t;
            sq = t;
        }
        r.s = sp * p.s + sq * q.s;
        r.v.x = sp * p.v.x + sq * q.v.x;
        r.v.y = sp * p.v.y + sq * q.v.y;
        r.v.z = sp * p.v.z + sq * q.v.z;
    }
    else
    {
        r.v.x = -p.v.y;
        r.v.y = p.v.x;
        r.v.z = -p.s;
        r.s = p.v.z;
        sp = sin ((1.0 - t) * HALFPI);
        sq = sin (t * HALFPI);
        r.v.x = sp * p.v.x + sq * r.v.x;
        r.v.y = sp * p.v.y + sq * r.v.y;
        r.v.z = sp * p.v.z + sq * r.v.z;
    }
    return r;
}
#endif

Matrix4d
Quaternion::QuaternionToMatrix4d () const
{
    Matrix4d m;

    m.m[0][0] = m_e[0] * m_e[0] + m_e[1] * m_e[1] -
        m_e[2] * m_e[2] - m_e[3] * m_e[3];
    m.m[1][0] = 2.0 * (m_e[1] * m_e[2] + m_e[0] * m_e[3]);
    m.m[2][0] = 2.0 * (m_e[1] * m_e[3] - m_e[0] * m_e[2]);

    m.m[0][1] = 2.0 * (m_e[1] * m_e[2] - m_e[0] * m_e[3]);
    m.m[1][1] = m_e[0] * m_e[0] + m_e[2] * m_e[2] -
        m_e[1] * m_e[1] - m_e[3] * m_e[3];
    m.m[2][1] = 2.0 * (m_e[2] * m_e[3] + m_e[0] * m_e[1]);

    m.m[0][2] = 2.0 * (m_e[1] * m_e[3] + m_e[0] * m_e[2]);
    m.m[1][2] = 2.0 * (m_e[2] * m_e[3] - m_e[0] * m_e[1]);
    m.m[2][2] = m_e[0] * m_e[0] + m_e[3] * m_e[3] -
        m_e[1] * m_e[1] - m_e[2] * m_e[2];

    m.m[0][3] = m.m[3][0] = 0.0;
    m.m[1][3] = m.m[3][1] = 0.0;
    m.m[2][3] = m.m[3][2] = 0.0;
    m.m[3][3] = 1.0;

    return m;
}


// TODO: review rotations (seem to be backwards, sign wise)

void
Matrix4d::Rotate (const Axis rotation_axis, const double theta)
{

    Matrix4d m (Matrix4d::Identity), tmp = *this;

    switch (rotation_axis)
    {

        case XAxis:
            m.mat (1, 1) = m.mat (2, 2) = cos (theta);
            m.mat (2, 1) = sin (theta);
            m.mat (1, 2) = -m.mat (2, 1);
            break;

        case YAxis:
            m.mat (0, 0) = m.mat (2, 2) = cos (theta);
            m.mat (0, 2) = sin (theta);
            m.mat (2, 0) = -m.mat (0, 2);
            break;

        case ZAxis:
            m.mat (0, 0) = m.mat (1, 1) = cos (theta);
            m.mat (1, 0) = sin (theta);
            m.mat (0, 1) = -m.mat (1, 0);
            break;
    }

    *this = tmp * m;
}


// Create a geographic to body transformation matrix based on
// Euler angles.

void
Matrix4d::EulerToMatrix4d (double dPhi, double dTheta, double dPsi)
// phi == roll, theta == pitch, psi == heading
{
    double sinPhi, cosPhi, sinTheta, cosTheta, sinPsi, cosPsi;

    sinPhi = sin (dPhi);
    cosPhi = cos (dPhi);
    sinTheta = sin (dTheta);
    cosTheta = cos (dTheta);
    sinPsi = sin (dPsi);
    cosPsi = cos (dPsi);

    mat (0, 0) = cosTheta * cosPsi;
    mat (1, 0) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
    mat (2, 0) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
    mat (0, 1) = cosTheta * sinPsi;
    mat (1, 1) = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
    mat (2, 1) = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
    mat (0, 2) = -sinTheta;
    mat (1, 2) = sinPhi * cosTheta;
    mat (2, 2) = cosPhi * cosTheta;
    mat (3, 0) = mat (3, 1) = mat (3, 2) = 0.0;
    mat (0, 3) = mat (1, 3) = mat (2, 3) = 0.0;
    mat (3, 3) = 1.0;
}

Matrix4d Matrix4d::Transpose () const
{
    int i, j;
    Matrix4d r;

    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            r.m[i][j] = m[j][i];
        }
    }
    return r;
}

Matrix4d::operator Quaternion ()
{
    double dPhi, dTheta, dPsi;
    Matrix4dToEuler (&dPhi, &dTheta, &dPsi);
    return Quaternion (dPhi, dTheta, dPsi);
};

// Apply the given transformation matrix m to this plane equation
// Note: for this to work, m_ABC must be a unit vector

PlaneEquation PlaneEquation::Transform (const Matrix4d & m) const
{
    // Generate a normal vector for the plane in the new coordinate system
    Vector3d vNewNormal = Normal ().Transform (m, Unaugmented);

    // Generate a point on the plane and transform it to the new coordinate system
    Vector3d vPointOnPlane = m * Vector3d (0, 0, -m_D / m_ABC.z);

    double d = -(vNewNormal.x * vPointOnPlane.x +
        vNewNormal.y * vPointOnPlane.y +
        vNewNormal.z * vPointOnPlane.z);

    return PlaneEquation (vNewNormal, d);
}

GeocentricCoordinates::GeocentricCoordinates (double ix,
double iy,
double iz):
Vector3d (ix, iy, iz)
{
};

// Convert Geocentric (ECF) coordinates to equivalent ECI coordinates

Vector3d GeocentricCoordinates::ECICoordinates (double GST_hours)
{
    Matrix4d
        m;
    m.init (Matrix4d::Identity);
    m.Rotate (ZAxis, DEGtoRAD (GST_hours * 15.0));

    return m * (*this);
}


GeodeticPosition::GeodeticPosition ()
{
    m_latitude_rad = 0.0;
    m_longitude_rad = 0.0;
    m_altitude_meters = 0.0;
}


GeodeticPosition::~GeodeticPosition ()
{

}

Matrix4d GeodeticPosition::ToNEDTransform () const
{
    Matrix4d ECF2NED (Matrix4d::Identity);

    double dLambda_rad;

    if (m_latitude_rad >= M_PI_2)
    {
        dLambda_rad = M_PI_2;
    }
    else if (m_latitude_rad <= -M_PI_2)
    {
        dLambda_rad = -M_PI_2;
    }
    else
    {
        dLambda_rad =
            atan ((1 - wgs84_f) * (1 -
            wgs84_f) *
            tan (m_latitude_rad));
    }

    ECF2NED.mat (0, 0) =
        -(cos (m_longitude_rad) * sin (dLambda_rad));
    ECF2NED.mat (0, 1) = -sin (m_longitude_rad);
    ECF2NED.mat (0, 2) =
        -(cos (dLambda_rad) * cos (m_longitude_rad));

    ECF2NED.mat (1, 0) =
        -sin (m_longitude_rad) * sin (dLambda_rad);
    ECF2NED.mat (1, 1) = cos (m_longitude_rad);
    ECF2NED.mat (1, 2) =
        -(cos (dLambda_rad) * sin (m_longitude_rad));

    ECF2NED.mat (2, 0) = cos (dLambda_rad);
    ECF2NED.mat (2, 1) = 0.0;
    ECF2NED.mat (2, 2) = -sin (dLambda_rad);

    return ECF2NED.Transpose ();
}

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

/*
 *  In the DIS 2.0 coordinate system:
 *
 *      positive Z axis is North;
 *      positive X axis points to 0N, 0E;
 *      positive Y axis points to 0N 90E.
 *
 *  So, North latitudes are positive; East longitudes are positive.
 *
 *  The world is considered a perfect ellipsoid based on the WGS84
 *  standard -- no correction is made to take into account height differences
 *  between the ellpsoid and the geoid.
 *
 *  "The Surveying Handbook", edited by Brinker and Minnick contains a decent
 *  discussion of the technical issues required to understand what's
 *  going on in this code.
 */

/*
 *  Shift location d meters on a given geodetic course (radians)
 */

void GeodeticPosition::UpdatePosition ( double cos_course, double sin_course,
                                        double d_meters)
{
    double n1, n2, m1;
    double sin_lat, sin_lat_sqr, tan_lat, sin_course_sqr;
    double delta_latitude, delta_longitude, d_sqr, cos_lat;
    double B, C, /* D, */ E, h, sin_newlat;

    /*  Increase our height to the height above the reference ellipsoid */

    double wgs84_a = WGS84_MAJOR + m_altitude_meters;

    sin_lat = sin (m_latitude_rad);
    sin_lat_sqr = sin_lat * sin_lat;
    cos_lat = cos (m_latitude_rad);
    tan_lat = sin_lat / cos_lat;
    sin_course_sqr = sin_course * sin_course;
    d_sqr = d_meters * d_meters;

    n1 = wgs84_a / sqrt (1.0 - WGS84_ECC_SQR * sin_lat_sqr);
    m1 = (wgs84_a * (1.0 - WGS84_ECC_SQR)) /
        pow (1.0 - WGS84_ECC_SQR * sin_lat_sqr, 1.5);

    B = 1.0 / m1;

    h = d_meters * B * cos_course;

    C = tan_lat / (2.0 * m1 * n1);

#ifdef notdef
    D = (3.0 * WGS84_ECC_SQR * sin_lat * cos_lat) /
        (2.0 * (1.0 - WGS84_ECC_SQR * sin_lat_sqr));
#endif

    E = (1.0 + 3.0 * tan_lat * tan_lat) *
        (1.0 - WGS84_ECC_SQR * sin_lat_sqr) / (6.0 * wgs84_a * wgs84_a);

    delta_latitude = d_meters * B * cos_course -
        d_sqr * C * sin_course_sqr - h * d_sqr * E * sin_course_sqr;

    m_latitude_rad += delta_latitude;
    if (m_latitude_rad > M_PI_2)
    {
        m_latitude_rad -= M_PI_2;
    }
    else if (m_latitude_rad < -M_PI_2)
    {
        m_latitude_rad += M_PI_2;
    }

    sin_newlat = sin (m_latitude_rad);

    n2 = wgs84_a / sqrt (1.0 - WGS84_ECC_SQR * sin_newlat * sin_newlat);

    delta_longitude = (d_meters * sin_course) / (n2 * cos (m_latitude_rad));

    m_longitude_rad += delta_longitude;
    if (m_longitude_rad > M_PI)
    {
        m_longitude_rad -= M_PI;
    }
    else if (m_longitude_rad < -M_PI)
    {
        m_longitude_rad += M_PI;
    }
}


/*
 *  Shift location d_meters meters on a given geodetic course (radians)
 *  returns new outbound heading correct for the new location in
 *  delta_course_rad
 */

void
GeodeticPosition::UpdatePositionEx (double cos_course, double sin_course,
                                    double d_meters,
                                    double *delta_course_rad)
{
    double n1, n2, m1;
    double sin_lat, sin_lat_sqr, tan_lat, sin_course_sqr;
    double delta_latitude, delta_longitude, d_sqr, cos_lat;
    double B, C, /* D, */ E, h, sin_newlat;
    double old_latitude, phi_m, sin_phi_m, cos_phi_m;

    /* arc-seconds per rad */
    const double rho = 206264.8062470964;

    /*  Increase our height to the height above the reference ellipsoid */

    double wgs84_a = WGS84_MAJOR + m_altitude_meters;

    sin_lat = sin (m_latitude_rad);
    sin_lat_sqr = sin_lat * sin_lat;
    cos_lat = cos (m_latitude_rad);
    tan_lat = sin_lat / cos_lat;
    sin_course_sqr = sin_course * sin_course;
    d_sqr = d_meters * d_meters;

    n1 = wgs84_a / sqrt (1.0 - WGS84_ECC_SQR * sin_lat_sqr);
    m1 = (wgs84_a * (1.0 - WGS84_ECC_SQR)) /
        pow (1.0 - WGS84_ECC_SQR * sin_lat_sqr, 1.5);

    B = 1.0 / m1;

    h = d_meters * B * cos_course;

    C = tan_lat / (2.0 * m1 * n1);

#ifdef notdef
    D = (3.0 * WGS84_ECC_SQR * sin_lat * cos_lat) /
        (2.0 * (1.0 - WGS84_ECC_SQR * sin_lat_sqr));
#endif

    E = (1.0 + 3.0 * tan_lat * tan_lat) *
        (1.0 - WGS84_ECC_SQR * sin_lat_sqr) / (6.0 * wgs84_a * wgs84_a);

    delta_latitude = d_meters * B * cos_course -
        d_sqr * C * sin_course_sqr - h * d_sqr * E * sin_course_sqr;

    old_latitude = m_latitude_rad;

    m_latitude_rad += delta_latitude;
    if (m_latitude_rad > M_PI_2)
    {
        m_latitude_rad -= M_PI_2;
    }
    else if (m_latitude_rad < -M_PI_2)
    {
        m_latitude_rad += M_PI_2;
    }

    phi_m = old_latitude + delta_latitude / 2.0;
    sin_phi_m = sin (phi_m);
    cos_phi_m = cos (phi_m);

    sin_newlat = sin (m_latitude_rad);

    n2 = wgs84_a / sqrt (1.0 - WGS84_ECC_SQR * sin_newlat * sin_newlat);

    delta_longitude = (d_meters * sin_course) / (n2 * cos (m_latitude_rad));

    *delta_course_rad =
        delta_longitude * sin_phi_m / cos (delta_latitude / 2.0) +
        delta_longitude * (sin_phi_m * cos_phi_m * cos_phi_m) / rho;

    m_longitude_rad += delta_longitude;
    if (m_longitude_rad > M_PI)
    {
        m_longitude_rad -= M_PI;
    }
    else if (m_longitude_rad < -M_PI)
    {
        m_longitude_rad += M_PI;
    }
}

#ifdef notnow
/*
 *  Convert cartesian geocentric coordinates into WGS84 geodetic lat/lon/z
 */
void
GeodeticPosition::GeocentricToWorldCoordinates (dis_world_coordinates * loc)
{
    double a_sqr = WGS84_MAJOR * WGS84_MAJOR, b_sqr =
        WGS84_MINOR * WGS84_MINOR;
    double w, x, x_sqr, z, delta_x, cos_x;
    double f, f_prime, w0, z0;

    w = sqrt (loc->x * loc->x + loc->y * loc->y);
    z = loc->z;

    /*
     *  x is the sine of the parametric latitude.  Use the sine of the geocentric
     *  latitude as the initial guess.
     */

    if (w == 0.0 && z == 0.0)
    {
        m_latitude_rad = 0.0;
        m_longitude_rad = 0.0;
        m_altitude_meters = 0.0;
        return;
    }

    x = z / sqrt (w * w + z * z);

    /*
     *  Compute x with accuracy that will yield a lat/lon accuracy of
     *  about 0.0001 arc-seconds (~ 0.10 foot).
     */

    for (delta_x = 1.0; fabs (delta_x) > 4.8E-10;)
    {

        x_sqr = x * x;

        cos_x = sqrt (1.0 - x_sqr);

        f = 2.0 * (WGS84_MAJOR * x * w - a_sqr * x * cos_x -
            WGS84_MINOR * cos_x * z + b_sqr * cos_x * x);

        f_prime =
            2.0 * (a_sqr + 2.0 * (a_sqr * x_sqr) - WGS84_MAJOR * w * x_sqr +
            b_sqr - 2.0 * b_sqr * x_sqr + WGS84_MINOR * x * z);

        delta_x = f / f_prime;
        x -= delta_x;
    }

    z0 = WGS84_MINOR * x;
    w0 = WGS84_MAJOR * sqrt (1.0 - x * x);

    m_altitude_meters = sqrt ((z - z0) * (z - z0) + (w - w0) * (w - w0));
    m_latitude_rad = atan (z0 / (w0 * (1.0 - WGS84_ECC_SQR)));
    m_longitude_rad = atan2 (loc->y, loc->x);
}
#endif


/*
 *  Convert WGS84 geodetic lat/lon/z into cartesian geocentric coordinates
 */

void GeodeticPosition::WorldCoordinatesToGeocentric (dis_world_coordinates * p) const
{
    double N, N1;
    double cos_latitude, sin_latitude;

    sin_latitude = sin (m_latitude_rad);
    cos_latitude = cos (m_latitude_rad);

    /*
     *  N is the length of the normal line segment from the surface to the
     *  spin axis.
     */

    N = WGS84_MAJOR / sqrt (1.0 -
        (WGS84_ECC_SQR * sin_latitude * sin_latitude));

    /*
     *  N1 lengthens the normal line to account for height above the surface
     */

    N1 = N + m_altitude_meters;

    p->x = N1 * cos_latitude * cos (m_longitude_rad);
    p->y = N1 * cos_latitude * sin (m_longitude_rad);
    p->z = (((WGS84_MINOR * WGS84_MINOR) / (WGS84_MAJOR * WGS84_MAJOR)) * N +
        m_altitude_meters) * sin_latitude;
}


/*
 * Convert double to form "DD.DDD"; pad with leading zero, if needed
 */

static char * special_format (const char *fmt, char *buf, double value)
{
    char tmpbuf[16];

    sprintf (tmpbuf, fmt, value);

    if (tmpbuf[1] == '.')
    {
        strcpy (buf, "0");
        strcat (buf, tmpbuf);
    }
    else
    {
        strcpy (buf, tmpbuf);
    }

    return buf;
}


char *
GeodeticPosition::LatitudeToString (char *s, double la,
LatLongDisplayFormat mode)
{

    int d, m;
    double dla, dmin, dsec;
    double round_dms = 1.0 / (36000.0 * 2.0);
    double round_dm = 1.0 / (600.0 * 2.0);
    int ns;
    char buf[16];

    round_dms = round_dm = 0.0;

    switch (mode)
    {

        case LLM_DMS:
            ns = (la >= 0.0) ? 'N' : 'S';
            dla = RADtoDEG (fabs (la)) + round_dms;
            d = (int) dla;
            dmin = (dla - (double) d) * 60.0;
            m = (int) dmin;
            dsec = (dmin - (double) m) * 60.0;

            sprintf (s, "%02d %02d %s %c",
                d, m, special_format ("%.3f", buf, dsec), ns);
            break;

        case LLM_DM:
            ns = (la >= 0.0) ? 'N' : 'S';
            dla = RADtoDEG (fabs (la)) + round_dm;
            d = (int) dla;
            dmin = (dla - (double) d) * 60.0;
            sprintf (s, "%02d %s %c", d, special_format ("%.3f", buf, dmin), ns);
            break;

        case LLM_D:
            ns = (la >= 0.0) ? 'N' : 'S';
            dla = RADtoDEG (fabs (la)) + 0.05;
            sprintf (s, "%.3f %c", dla, ns);
            break;

        case LLM_SIGNED_D:
            sprintf (s, "%.3f", RADtoDEG (la));
            break;
    }

    return s;

}


char *
GeodeticPosition::LatitudeToString (char *s, LatLongDisplayFormat mode) const
{
    return LatitudeToString (s, m_latitude_rad, mode);
}


char *
GeodeticPosition::LongitudeToString (char *s, double lo,
LatLongDisplayFormat mode)
{

    int d, m;
    double dlo, dmin, dsec;
    double round_dms = 1.0 / (36000.0 * 2.0);
    double round_dm = 1.0 / (600.0 * 2.0);
    int ew;
    char buf[16];

    round_dms = round_dm = 0.0;

    ew = (lo >= 0.0) ? 'E' : 'W';

    switch (mode)
    {

        case LLM_DMS:
            dlo = RADtoDEG (fabs (lo)) + round_dms;
            d = (int) dlo;
            dmin = (dlo - (double) d) * 60.0;
            m = (int) dmin;
            dsec = (dmin - (double) m) * 60.0;
            sprintf (s, "%03d %02d %s %c", d, m,
                special_format ("%.3f", buf, dsec), ew);
            break;

        case LLM_DM:
            dlo = RADtoDEG (fabs (lo)) + round_dm;
            d = (int) dlo;
            dmin = (dlo - (double) d) * 60.0;
            sprintf (s, "%03d %s %c", d, special_format ("%.3f", buf, dmin), ew);
            break;

        case LLM_D:
            dlo = RADtoDEG (fabs (lo)) + 0.05;
            sprintf (s, "%.3f %c", dlo, ew);
            break;

        case LLM_SIGNED_D:
            sprintf (s, "%.1f", RADtoDEG (lo));
            break;

    }

    return s;

}


char *
GeodeticPosition::LongitudeToString (char *s, LatLongDisplayFormat mode) const
{
    return LongitudeToString (s, m_longitude_rad, mode);
}


#define STATE_INITIAL   0
#define STATE_WORD  1
#define STATE_INTEGER   2
#define STATE_FLOAT 3

typedef enum
{
    EndOfFile,
    TOKEN_FLOAT,
    TOKEN_LONG,
    TOKEN_DASH,
    TOKEN_NORTH,
    TOKEN_SOUTH,
    TOKEN_EAST,
    TOKEN_WEST,
    TOKEN_GEOTIFF_DEGREES,
    TOKEN_GEOTIFF_MINUTES,
    TOKEN_GEOTIFF_SECONDS,
    TOKEN_LPAREN,
    TOKEN_RPAREN,
    TOKEN_COMMA
}


token_id;

typedef union
{
    double double_value;
    long long_value;
}


lex_val;

static lex_val lex_value;

struct lex_record
{
    char *s;
    FILE *f;
    int lookahead_valid;
    int lookahead;
    int stack_top;
    lex_val value_stack[16];
};

static int
input_char (struct lex_record *p)
{
    int val;

    if (p->lookahead_valid)
    {
        p->lookahead_valid = 0;
        val = p->lookahead;
    }
    else if (p->s)
    {
        val = *(p->s)++;
    }
    else
    {
        //              val = fgetc(p->f);
        val = 0;                                  // cannot happen here!
    }
    return val;
}


#define push_value(p, type, val) \
p->value_stack[p->stack_top++].type = val

#define pop_value(p, type) (p->value_stack[--p->stack_top].type)

#define unput(p, c) { p->lookahead = c; p->lookahead_valid = 1; }

#define InitializeLexRecord(p)  { p->lookahead_valid = 0; }

static char token[256];
static int token_length = 0;

static token_id
NextTokenx (struct lex_record *p)
{
    int c, state = STATE_INITIAL;

    token_length = 0;

    while ((c = input_char (p)) != EOF)
    {

        switch (state)
        {

            case STATE_INITIAL:

                if (isspace (c))
                {
                    continue;
                }
                else if (isdigit (c))
                {
                    token[token_length++] = (char) c;
                    state = STATE_INTEGER;
                }
                else if (c == '.')
                {
                    token[token_length++] = (char) c;
                    state = STATE_FLOAT;
                }
                else
                {
                    token[0] = (char) c;
                    token[1] = '\0';
#ifdef DEBUG
                    printf ("other %s\n", token);
#endif
                    switch (c)
                    {
                        case '-':
                            return TOKEN_DASH;
                        case '(':
                            return TOKEN_LPAREN;
                        case ')':
                            return TOKEN_RPAREN;
                        case 'n':
                        case 'N':
                            return TOKEN_NORTH;
                        case 'e':
                        case 'E':
                            return TOKEN_EAST;
                        case 's':
                        case 'S':
                            return TOKEN_SOUTH;
                        case 'w':
                        case 'W':
                            return TOKEN_WEST;
                            /*
                             *  invalid character
                             */
                        default:
                            return EndOfFile;
                    }
                }
                break;

            case STATE_INTEGER:
            case STATE_FLOAT:
                if (isspace (c) ||
                    c == '-' ||
                    toupper (c) == 'N' ||
                    toupper (c) == 'S' ||
                    toupper (c) == 'W' || toupper (c) == 'E')
                {
                    token[token_length] = '\0';
                    unput (p, c);
                    if (state == STATE_INTEGER)
                    {
                        lex_value.long_value = strtol (token, NULL, 0);
                        return TOKEN_LONG;
                    }
                    else
                    {
                        lex_value.double_value = strtod (token, NULL);
                        return TOKEN_FLOAT;
                    }
                }
                else
                {
                    if (c == '.')
                    {
                        state = STATE_FLOAT;
                    }
                    token[token_length++] = (char) c;
                }
                break;

            default:
                token[token_length++] = (char) c;
                break;
        }
    }

    return EndOfFile;
}


static token_id
NextToken (struct lex_record *p)
{
    token_id t;

    t = NextTokenx (p);

#ifdef DEBUG
    printf ("token %s\n", token);
#endif
    return t;
}


static int
ParseLatitude (struct lex_record *p)
{
    double x = 0.0;
    double divider = 1.0;
    int int_valid = 1;
    token_id t;

    t = NextToken (p);
    for (;;)
    {
        switch (t)
        {
            case TOKEN_NORTH:
                lex_value.double_value = x;
                return 0;

            case TOKEN_SOUTH:
                lex_value.double_value = -x;
                return 0;

            case TOKEN_LONG:
                if (int_valid)
                {
                    x += lex_value.long_value / divider;
                    divider *= 60.0;
                    t = NextToken (p);
                    if (t == TOKEN_DASH)
                    {
                        t = NextToken (p);
                    }
                }
                else
                {
                    return -1;
                }
                break;

            case TOKEN_FLOAT:
                int_valid = 0;
                x += lex_value.double_value / divider;
                divider *= 60.0;
                t = NextToken (p);
                if (t == TOKEN_DASH)
                {
                    t = NextToken (p);
                }
                break;
            default:
                return -1;
        }
    }
}


static int
ParseLongitude (struct lex_record *p)
{
    double x = 0.0;
    double divider = 1.0;
    int t, int_valid = 1;

    t = NextToken (p);
    for (;;)
    {
        switch (t)
        {
            case TOKEN_EAST:
                lex_value.double_value = x;
                return 0;

            case TOKEN_WEST:
                lex_value.double_value = -x;
                return 0;

            case TOKEN_LONG:
                if (int_valid)
                {
                    x += lex_value.long_value / divider;
                    divider *= 60.0;
                    t = NextToken (p);
                    if (t == TOKEN_DASH)
                    {
                        t = NextToken (p);
                    }
                }
                else
                {
                    return -1;
                }
                break;

            case TOKEN_FLOAT:
                int_valid = 0;
                x += lex_value.double_value / divider;
                divider *= 60.0;
                t = NextToken (p);
                if (t == TOKEN_DASH)
                {
                    t = NextToken (p);
                }
                break;

            default:
                return -1;
        }
    }
}

char *
GeodeticPosition::StringToLatLong (char *s)
{
    struct lex_record p;

    p.s = s;
    p.lookahead_valid = 0;

    if (ParseLatitude (&p) != 0)
    {
        return 0;
    }
    m_latitude_rad = DEGtoRAD (lex_value.double_value);

    if (ParseLongitude (&p) != 0)
    {
        return 0;
    }
    m_longitude_rad = DEGtoRAD (lex_value.double_value);
    m_altitude_meters = 0.0;
    return p.s;
}


GeodeticPosition::operator GeocentricCoordinates () const
{
    dis_world_coordinates p;
    WorldCoordinatesToGeocentric (&p);
    return GeocentricCoordinates (p.x, p.y, p.z);
}

const double re = 6378137.0;                      /* earth radius at equator, meters */
const double GM = 0.3986004418e15;                /* m^3 s^-2 */
const double J2 = 1.08263e-03;                    /* gravitational harmonic constant */

GravityModel::GravityModel ()
{
}

GravityModel::~GravityModel ()
{
}

Vector3d
GravityModel::ComputeECIGravityVector (const Vector3d & p_meters) const
{
    return ComputeECIGravityVectorEx (p_meters, p_meters.z /
        sqrt (p_meters.x * p_meters.x +
        p_meters.y * p_meters.y +
        p_meters.z * p_meters.z));
}

Vector3d
GravityModel::ComputeECIGravityVectorEx (const Vector3d & p_meters, double dSinGeocentricLatitude)
const
{
    double dSqr = dSinGeocentricLatitude * dSinGeocentricLatitude;
    double dpMag_meters = p_meters.Magnitude ();
    double dREPSqr = (re / dpMag_meters) * (re / dpMag_meters);
    Vector3d p_bar, g;

    p_bar.x = p_meters.x * (1.0 + 1.5 * J2 * dREPSqr * (1.0 - 5.0 * dSqr));
    p_bar.y = p_meters.y * (1.0 + 1.5 * J2 * dREPSqr * (1.0 - 5.0 * dSqr));
    p_bar.z = p_meters.z * (1.0 + 1.5 * J2 * dREPSqr * (3.0 - 5.0 * dSqr));

    g = p_bar * (-GM / (dpMag_meters * dpMag_meters * dpMag_meters));

    return g;
}