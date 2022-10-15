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
#ifndef GEODESY_H
#define GEODESY_H

#include <fstream>
#include <istream>
#include <ostream>
#include <memory.h>
#include <math.h>

#ifndef M_PI
const double M_PI   = 3.14159265358979323846;
const double M_PI_2 = 1.57079632679489661923;
const double M_PI_4 = 0.78539816339744830962;
#endif

#if !defined (DEGtoRAD)
#define DEGtoRAD(theta) ((theta) * M_PI / 180.0)
#define RADtoDEG(theta) ((theta) * 180.0 / M_PI)
#endif

#define FEETtoMETERS(x)  ((x)*0.3048)
#define METERStoFEET(x)  ((x)*3.28084)

/**
 * A Lightweight Geodesy and Vector Math Library based on my (so far unpublished) DSO simulation libraries
 */
namespace Geodesy {

typedef enum _Axis {
	XAxis,
	YAxis,
	ZAxis
	} Axis;

typedef enum _AugmetationType {
	Augmented,
	Unaugmented
} AugmentationType;

class Matrix4d;

/**
 * A three-dimensional 'double' vector class
 */
class Vector3d {
public:
  double	x, y, z; //< cartesian coordinates

	Vector3d (double ix = 0.0, double iy = 0.0, double iz = 0.0)
		{ x = ix; y = iy; z = iz; };


	/**
	 * @defgroup ops Vector3d C++ Operators
	 * C++ operators defined for vectors
	 */

	/**
	 * @defgroup other Vector3d Miscellaneous Methods
	 * Other vector operations
	 */

	/**
	 * @ingroup ops
	 * Equality operator
	 */
	inline Vector3d operator ==(const Vector3d& b) const
	    { 
		return (x == b.x) && (y == b.y) && (z == b.z); 
	    };

	/**
	 * @ingroup ops
	 * Inequality operator
	 */
	inline Vector3d operator !=(const Vector3d& b) const
	    { 
		return ! ((x == b.x) && (y == b.y) && (z == b.z)); 
	    };

	/**
	 * @ingroup ops
	 * Unary minus operator
	 */
	inline Vector3d operator -() const
		{ return Vector3d ( - x, - y, - z ); };

	/**
	 * @ingroup ops
	 * vector addition
	 */
	inline
	Vector3d operator +(const Vector3d& b) const
		{ return Vector3d(x + b.x, y + b.y, z + b.z); };

	/**
	 * @ingroup ops
	 * vector addition
	 */
	inline
	Vector3d& operator +=(const Vector3d& b)
		{ x += b.x; y += b.y; z += b.z; return (*this); }

	/**
	 * @ingroup ops
	 * vector subtraction
	 */
	inline
	Vector3d operator -(const Vector3d& b) const {
		return Vector3d(x - b.x, y - b.y, z - b.z);
	}

	/**
	 * @ingroup ops
	 * vector subtraction
	 */
	inline
	Vector3d& operator -=(const Vector3d& b) {
		x -= b.x; y -= b.y, z -= b.z;
		return (*this);
	}

	/**
	 * vector cross product (deprecated, use CrossProduct)
	 * @see CrossProduct
	 */
	inline
	Vector3d Cross(const Vector3d &b) const
		{
			return Vector3d (
				y * b.z - z * b.y,
				z * b.x - x * b.z,
				x * b.y - y * b.x );
		};

	/**
	 * @ingroup other
	 * vector cross product
	 */
	inline
	Vector3d CrossProduct(const Vector3d &b) const
		{
			return Vector3d (
				y * b.z - z * b.y,
				z * b.x - x * b.z,
				x * b.y - y * b.x );
		};

	/**
	 * @ingroup ops
	 * vector multiplication
	 */
	inline
	Vector3d operator * (const Vector3d& b) const
		{ return Vector3d(x * b.x, y * b.y, z * b.z); };

	inline
	Vector3d& operator *=(double s)
		{ x *= s; y *= s; z *= s; return (*this);};
	
	/**
	 * @ingroup ops
	 * multiply a vector by a scalar
	 */
	inline
	Vector3d operator * (double s) const
		{ return Vector3d(x * s, y * s, z * s); };

	// Various transformation functions

	Vector3d Transform(const Matrix4d& m,
			   AugmentationType t = Augmented) const;

	/**
	 * @ingroup ops
	 * divide a vector by a scalar
	 */
	inline
	Vector3d operator / (double s) const
		{ return Vector3d(x / s, y / s, z / s); };
	
	
	inline
	Vector3d& operator /=(double s)
		{ x /= s; y /= s; z /= s; return (*this); };
	
	/**
	 * @ingroup ops
	 * vector division
	 */
	inline
	Vector3d operator / (const Vector3d& b) const
		{ return Vector3d (x / b.x, y / b.y, z / b.z); };

	/**
	 * @ingroup other
	 * vector dot product
	 */
	inline
	double DotProduct(const Vector3d& b) const
		{ return x * b.x + y * b.y + z * b.z; };

	/**
	 * @ingroup ops
	 * vector dot product as a C++ operator
	 */
	inline
	double operator ^(const Vector3d& b) const
		{ return x * b.x + y * b.y + z * b.z; };

	/**
	 * @ingroup other
	 * define a stream output function
	 */

	inline friend std::ostream & operator<< (std::ostream &o, 
						 const Vector3d &v)
		{
			o << "{ " << v.x << ", " << v.y << ", " << v.z << " }";
			return o;
		};
		
	/**
	 * @ingroup other
	 * compute the unit vector of a given vector
	 */
	inline Vector3d UnitVector () const
		{
			double mag = sqrt (x * x + y * y + z * z);
			return Vector3d (x / mag, y / mag, z / mag);
		};
		
	/**
	 * @ingroup other
	 * compute the length of a vector
	 */
	inline double Magnitude () const
		{
			return sqrt(x * x + y * y + z * z);
		};
};

static double ident[4][4] = {{1, 0, 0, 0}, 
			     {0, 1, 0, 0}, 
			     {0, 0, 1, 0}, 
			     {0, 0, 0, 1}};

static double  zero[4][4] = {{0, 0, 0, 0}, 
			     {0, 0, 0, 0}, 
			     {0, 0, 0, 0}, 
			     {0, 0, 0, 0}};

class Quaternion;

/**
 * A 4x4 double matrix
 */
class Matrix4d {
public:
	Matrix4d Transpose(void) const;
	enum initial_state {
	    Zero,
	    Identity
	    };

	Matrix4d() { init(Identity); }
	Matrix4d(const enum initial_state state) { init(state); };
	Matrix4d(double m00, double m01, double m02,
		 double m10, double m11, double m12,
		 double m20, double m21, double m22);
	Matrix4d(const double latitude_rad, const double longitude_rad);
	friend class Quaternion;
	Matrix4d operator * (const Matrix4d &b) const;
	Vector3d operator * (const class Vector3d &b) const;

	/**
	 * convert aerospace Euler angles (XYZ rotation order)
	 * to an equivalent direction cosine matrix.
	 */
	void EulerToMatrix4d (double dPhi_rad, 
			      double dTheta_rad, 
			      double dPsi_rad);

	void Matrix4dToEuler (double *dPhi_rad, 
			      double *dTheta_rad, 
			      double *dPsi_rad) const;

	inline double & mat(const int i, const int j) const
		// mathematically: row i+1, column j+1
		{ return (double &)(m[j][i]); }
	inline double & mat1(const int i, const int j) const
		// mathematically: row i, column j
		{ return (double &)(m[j-1][i-1]); }
	friend std::ostream & operator << (std::ostream &o, 
					   const Matrix4d &m);
	void Rotate (const Axis rotation_axis, const double theta_rad);
	double	m[4][4];
	inline void init(const enum initial_state state)
	{
		if (state == Zero)
			memcpy (&m, zero, sizeof(m));
		else
			memcpy (&m, ident, sizeof(m));
	}

	void Translate( const Vector3d & x );
	void Translate( double x, double y, double z );
	void Scale( const Vector3d & x );
	void Scale( double x, double y, double z );

	/**
	 * convert a cosine matrix to an equivalent Quaternion
	 */
	operator Quaternion();
};

/**
 *
 * Quaternion class -- implements Quaternions
 *
 */

class Quaternion {
public:

	// Constructors

        /**
         * Quaternion from Euler angles (aerospace sequence)
         */
	Quaternion (
		double dPhi_rad = 0.0,
		double dTheta_rad = 0.0,
		double dPsi_rad = 0.0)
		{	
		    EulerToQuaternion ( dPhi_rad, dTheta_rad, dPsi_rad ); 
		}

	/**
         * Quaternion initialized directly
         */
	Quaternion (double e0, double e1, double e2, double e3)
		{ m_e[0] = e0; m_e[1] = e1; m_e[2] = e2; m_e[3] = e3; }
	/**
         * Quaternion initialized directly from four element vector
         */
	Quaternion (const double e[4])
		{ m_e[0] = e[0]; m_e[1] = e[1]; m_e[2] = e[2]; m_e[3] = e[3]; }

	/**
         * Cast quaternion to an equivalent transformation matrix
         */
	Matrix4d QuaternionToMatrix4d (void) const;

	/**
         * Cast quaternion to an equivalent transformation matrix
         */
	operator Matrix4d()
	{ return QuaternionToMatrix4d(); };

	/**
         * Convert Euler angles to a quaternion
         */
	void EulerToQuaternion (double phi_rad,
				double theta_rad, 
				double psi_rad) {
		double sPsi, cPsi, sTheta, cTheta, sPhi, cPhi;
		sPsi =   sin( psi_rad / 2. );
		cPsi =   cos( psi_rad / 2. );
		sTheta = sin( theta_rad / 2. );
		cTheta = cos( theta_rad / 2. );
		sPhi =   sin( phi_rad / 2. );
		cPhi =   cos( phi_rad / 2. );
		m_e[0] =   cPsi * cTheta * cPhi + sPsi * sTheta * sPhi;
		m_e[1] =   cPsi * cTheta * sPhi - sPsi * sTheta * cPhi;
		m_e[2] =   cPsi * sTheta * cPhi + sPsi * cTheta * sPhi;
		m_e[3] = - cPsi * sTheta * sPhi + sPsi * cTheta * cPhi;
	}

	/**
	 * Convert quaternion to equivalent Euler angles
         */
	void QuaternionToEuler (
		double *dPhi_rad,
		double *dTheta_rad,
		double *dPsi_rad) const {
		*dTheta_rad = asin ( -2.0 * ( m_e[1] * m_e[3] - m_e[0] * 
					      m_e[2] ));
		*dPsi_rad   = atan2 (2.0 * (m_e[1] * m_e[2] + m_e[0] * m_e[3]),
			m_e[0] * m_e[0] + m_e[1] * m_e[1] - m_e[2] * m_e[2] - 
				     m_e[3] * m_e[3] );
		*dPhi_rad   = atan2 (2.0 * (m_e[2] * m_e[3] + m_e[0] * m_e[1]),
			m_e[0] * m_e[0] + m_e[3] * m_e[3] - m_e[1] * m_e[1] - 
				     m_e[2] * m_e[2] );
	};

	/**
     * Generate Quaternion derivatives from body angular rates
     */
	Quaternion EulerRatesToQuaternionRates (double p, 
						double q, 
						double r) const {
		return Quaternion(
			- 0.5 * (m_e[1] * p + m_e[2] * q + m_e[3] * r),
			  0.5 * (m_e[0] * p + m_e[2] * r - m_e[3] * q),
			  0.5 * (m_e[0] * q + m_e[3] * p - m_e[1] * r),
			  0.5 * (m_e[0] * r + m_e[1] * q - m_e[2] * p)
			);
	};

	/**
	 * Get the E-th element of the quaternion (0 .. 3)
	 */
	double GetE (int nIndex) const
	{ return (nIndex < 0 || nIndex > 3) ? 0.0 : m_e[nIndex]; }

	double GetW() const { return m_e[0]; };
	double GetX() const { return m_e[1]; };
	double GetY() const { return m_e[2]; };
	double GetZ() const { return m_e[3]; };

	void SetW( double d ) { m_e[0] = d; };
	void SetX( double d ) { m_e[1] = d; };
	void SetY( double d ) { m_e[2] = d; };
	void SetZ( double d ) { m_e[3] = d; };

	/**
	 * Convert Quaternion derivative to equivalent axis rates
	 */
	void QuaternionRatesToAxisRates (Quaternion &ori, 
					 double *p, 
					 double *q, 
					 double *r) const
	{
		*p = 2.0 * (ori.GetE(0) * m_e[1] + ori.GetE(3) * m_e[2] -
			ori.GetE(2) * m_e[3] - ori.GetE(1) * m_e[0] );
		*q = 2.0 * (-ori.GetE(3) * m_e[1] + ori.GetE(0) * m_e[2] +
			ori.GetE(1) * m_e[3] - ori.GetE(2) * m_e[0] );
		*r = 2.0 * (ori.GetE(2) * m_e[1] - ori.GetE(1) * m_e[2] +
			ori.GetE(0) * m_e[3] - ori.GetE(3) * m_e[0] );
	}

	/**
	 *  Generates top ("X") row of a Quaternion to Matrix conversion
	 */
	Vector3d XVector () const {
		return Vector3d (
			m_e[0] * m_e[0] + m_e[1] * m_e[1] -
		        m_e[2] * m_e[2] - m_e[3] * m_e[3],
			2.0 * (m_e[1] * m_e[2] - m_e[0] * m_e[3]),
			2.0 * (m_e[1] * m_e[3] + m_e[0] * m_e[2])
			);
	}

	/**
	 * Quaternion multiplication.
	 * With thanks to Dr. William Rowan Hamilton.
	 */
	inline
	Quaternion operator * (const Quaternion &b) const
		{
			Vector3d av(GetE(1), GetE(2), GetE(3));
			Vector3d bv(b.GetE(1), b.GetE(2), b.GetE(3));
			Vector3d v = bv *  GetE(0) +  av * b.GetE(0) + 
			  av.CrossProduct(bv);
			return Quaternion (GetE(0) * b.GetE(0) - (av ^ bv),
				v.x, v.y, v.z);
		}

	/**
     * Compute the conjugate of the given quaternion
	 */
	inline
	Quaternion Conjugate () const
		{ return Quaternion (m_e[0], - m_e[1], - m_e[2], - m_e[3]); }

	/**
	 * negate a quaternion
	 */
	inline Quaternion operator -() const
	  { return Quaternion (- m_e[0], - m_e[1], - m_e[2], - m_e[3]); };

	inline friend std::ostream & operator<< (std::ostream &o, 
						 const Quaternion &q)
		{
			o << "( " << q.m_e[0] << ", " <<
				         q.m_e[1] << ", " <<
		                 q.m_e[2] << ", " <<
						 q.m_e[3] << " )";
			return o;
		}

private:
	double	m_e[4];
};


extern Matrix4d identity_matrix;
extern Matrix4d zero_matrix;

class PlaneEquation
{

protected:
	// A x + B y + C z + D = 0
	Vector3d	m_ABC;
	double		m_D;

public:
	PlaneEquation( )
	{
		m_ABC = Vector3d(0,0,-1);
		m_D = 0.0;
	}
	PlaneEquation( const Vector3d & ABC, double D )
	{ m_ABC = ABC; m_D = D; }
	PlaneEquation( double A, double B, double C, double D )
	{ m_ABC = Vector3d( A, B, C); m_D = D; }
	const Vector3d & Normal() const { return m_ABC; }

	double DistanceFromPlane( const Vector3d& v ) const
	{ return (m_ABC.x * v.x + m_ABC.y * v.y + m_ABC.z * v.z + m_D) / 
	    m_ABC.Magnitude(); }
	PlaneEquation Transform (const Matrix4d & m ) const;
};

#define SECtoRAD(x) ((x) * M_PI / (180.0 * 3600.0))

#define WGS84_MAJOR 6378137.0                     /* meters */
#define WGS84_MINOR 6356752.3142                  /* meters */
#define WGS84_ECC   0.081819190928906199466       /* eccentricity */
#define WGS84_ECC_SQR   0.006694380004260806515   /* eccentricity squared */
#define wgs84_f (1/298.257223563)                 /* WGS-84 earth flattening parameter */

typedef struct _dis_world_coordinates
{
    double x, y, z;
} dis_world_coordinates;

class GeocentricCoordinates;

class GeodeticPosition
{
    public:
        GeodeticPosition ();
        virtual ~ GeodeticPosition ();

        typedef enum
        {
            LLM_DMS,                          /* dd mm ss.s [EWNS] */
            LLM_DM,                           /* dd mm [EWNS] */
            LLM_D,                            /* dd [EWNS] */
            LLM_SIGNED_D
        }
        LatLongDisplayFormat;

		Matrix4d ToNEDTransform () const;

        void UpdatePosition (double cos_course,
            double sin_course, double d_meters);

        void UpdatePositionEx (double cos_course,
            double sin_course,
            double d_meters, double *delta_course_rad);

        void GeocentricToWorldCoordinates (dis_world_coordinates * loc);

        void WorldCoordinatesToGeocentric (dis_world_coordinates * p) const;

        char *StringToLatLong (char *s);

        operator GeocentricCoordinates () const;

        char *LatitudeToString (char *s, LatLongDisplayFormat mode) const;
        char *LongitudeToString (char *s, LatLongDisplayFormat mode) const;

    public:
        double m_latitude_rad;
        double m_longitude_rad;
        double m_altitude_meters;

    public:
        static char *LatitudeToString (char *s, double la,
            LatLongDisplayFormat mode);
        static char *LongitudeToString (char *s, double lo,
            LatLongDisplayFormat mode);
};

class GeocentricCoordinates:public Vector3d
{
    public:
        GeocentricCoordinates (double ix = 0.0,
            double iy = 0.0,
            double iz = 0.0);
        GeocentricCoordinates (const Vector3d & rhs)
        {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }

        operator GeodeticPosition () const
        {
            GeodeticPosition a;
            dis_world_coordinates b;
            b.x = x;
            b.y = y;
            b.z = z;
            a.GeocentricToWorldCoordinates (&b);
            return a;
        }

        Vector3d ECICoordinates (double GST);
};

class GravityModel
{
    public:
        GravityModel();
        virtual ~GravityModel();

        // return gravity acceleration vector (newtons*sec^-2) at the given
        // ECI position coordinates (meters) */

        virtual Vector3d ComputeECIGravityVector( const Vector3d & p_meters ) const;

        // return gravity acceleration vector (newtons*sec^-2) at the given
        // ECI position coordinates (meters) and a pre-calculated sin(geoLatitude)

        virtual Vector3d ComputeECIGravityVectorEx( const Vector3d & p_meters,
            double dSinGeocentricLatitude ) const;

};

}
#endif
