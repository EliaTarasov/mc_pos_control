#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>

#define PX4_ISFINITE(x) std::isfinite(x)
  
namespace mc_pos_control {

  typedef double scalar_t;
  typedef Eigen::Matrix<scalar_t, 4, 1> vec4; /// Vector in R4
  typedef Eigen::Matrix<scalar_t, 3, 1> vec3; /// Vector in R3
  typedef Eigen::Matrix<scalar_t, 2, 1> vec2; /// Vector in R2
  typedef Eigen::Matrix<scalar_t, 3, 3> mat3; /// Matrix in R3
  typedef Eigen::Quaternion<scalar_t> quat; /// Member of S4

  struct PositionControlStates {
    vec3 position;
    vec3 velocity;
    vec3 acceleration;
    scalar_t yaw;
  };

  struct Constraints {
    scalar_t yawspeed; // in radians/sec
    scalar_t speed_xy; // in meters/sec
    scalar_t speed_up; // in meters/sec
    scalar_t speed_down; // in meters/sec
    scalar_t tilt; // in radians [0, PI]
  };

  /**
    * Quaternion for rotation between ENU and NED frames
    *
    * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
    * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
    */
  const quat q_ng = quat(0, 0.70711, 0.70711, 0);

  /**
    * Quaternion for rotation between body FLU and body FRD frames
    *
    * FLU to FRD: +PI rotation about X(forward)
    * FRD to FLU: -PI rotation about X(forward)
    */
  const quat q_rb = quat(0, 1, 0, 0);

  // Type-safe signum function
  template<typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
  }

  // Type-safe signum function with zero treated as positive
  template<typename T> int signNoZero(T val) {
    return (T(0) <= val) - (val < T(0));
  }

  template<typename Scalar>
  static inline constexpr const Scalar &constrain(const Scalar &val, const Scalar &min_val, const Scalar &max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
  }

  /*
   * So called exponential curve function implementation.
   * It is essentially a linear combination between a linear and a cubic function.
   * @param value [-1,1] input value to function
   * @param e [0,1] function parameter to set ratio between linear and cubic shape
   * 		0 - pure linear function
   * 		1 - pure cubic function
   * @return result of function output
   */
  template<typename T> const T expo(const T &value, const T &e) {
    T x = constrain(value, (T) - 1, (T) 1);
    T ec = constrain(e, (T) 0, (T) 1);
    return (1 - ec) * x + ec * x * x * x;
  }

  /*
   * So called SuperExpo function implementation.
   * It is a 1/(1-x) function to further shape the rc input curve intuitively.
   * I enhanced it compared to other implementations to keep the scale between [-1,1].
   * @param value [-1,1] input value to function
   * @param e [0,1] function parameter to set ratio between linear and cubic shape (see expo)
   * @param g [0,1) function parameter to set SuperExpo shape
   * 		0 - pure expo function
   * 		0.99 - very strong bent curve, stays zero until maximum stick input
   * @return result of function output
   */
  template<typename T> const T superexpo(const T &value, const T &e, const T &g) {
    T x = constrain(value, (T) - 1, (T) 1);
    T gc = constrain(g, (T) 0, (T) 0.99);
    return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
  }

  /*
   * Deadzone function being linear and continuous outside of the deadzone
   * 1                ------
   *                /
   *             --
   *           /
   * -1 ------
   *        -1 -dz +dz 1
   * @param value [-1,1] input value to function
   * @param dz [0,1) ratio between deazone and complete span
   * 		0 - no deadzone, linear -1 to 1
   * 		0.5 - deadzone is half of the span [-0.5,0.5]
   * 		0.99 - almost entire span is deadzone
   */
  template<typename T> const T deadzone(const T &value, const T &dz) {
    T x = constrain(value, (T) - 1, (T) 1);
    T dzc = constrain(dz, (T) 0, (T) 0.99);
    // Rescale the input such that we get a piecewise linear function that will be continuous with applied deadzone
    T out = (x - sign(x) * dzc) / (1 - dzc);
    // apply the deadzone (values zero around the middle)
    return out * (fabsf(x) > dzc);
  }

  template<typename T> const T expo_deadzone(const T &value, const T &e, const T &dz) {
    return expo(deadzone(value, dz), e);
  }

  template <typename T> inline T max(T val1, T val2) {
    return (val1 > val2) ? val1 : val2;
  }

  template <typename T> inline T min(T val1, T val2) {
    return (val1 < val2) ? val1 : val2;
  }

  template<typename T> constexpr T radians(const T degrees) {
    return degrees * (static_cast<T>(M_PI) / static_cast<T>(180));
  }

  /**
    * Constructor from euler angles
    *
    * This sets the transformation matrix from frame 2 to frame 1 where the rotation
    * from frame 1 to frame 2 is described by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
    *
    *
    * @param euler euler angle instance
  */
  inline mat3 euler2dcm(const vec3 &euler) {
    mat3 dcm;

    scalar_t cosPhi = cos(euler(0));
    scalar_t sinPhi = sin(euler(0));
    scalar_t cosThe = cos(euler(1));
    scalar_t sinThe = sin(euler(1));
    scalar_t cosPsi = cos(euler(2));
    scalar_t sinPsi = sin(euler(2));

    dcm(0, 0) = cosThe * cosPsi;
    dcm(0, 1) = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
    dcm(0, 2) = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

    dcm(1, 0) = cosThe * sinPsi;
    dcm(1, 1) = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
    dcm(1, 2) = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

    dcm(2, 0) = -sinThe;
    dcm(2, 1) = sinPhi * cosThe;
    dcm(2, 2) = cosPhi * cosThe;

    return dcm;
  }

  inline vec3 dcm2vec(const mat3& dcm) {
    scalar_t phi_val = atan2(dcm(2, 1), dcm(2, 2));
    scalar_t theta_val = asin(-dcm(2, 0));
    scalar_t psi_val = atan2(dcm(1, 0), dcm(0, 0));
    scalar_t pi = M_PI;

    if (fabs(theta_val - pi / 2) < 1.0e-3) {
      phi_val = 0.0;
      psi_val = atan2(dcm(1, 2), dcm(0, 2));
    } else if (fabs(theta_val + pi / 2) < 1.0e-3) {
      phi_val = 0.0;
      psi_val = atan2(-dcm(1, 2), -dcm(0, 2));
    }
    return vec3(phi_val, theta_val, psi_val);
  }

  inline mat3 quat2dcm(const quat& q) {
    mat3 dcm;
    scalar_t a = q.w();
    scalar_t b = q.x();
    scalar_t c = q.y();
    scalar_t d = q.z();
    scalar_t aSq = a * a;
    scalar_t bSq = b * b;
    scalar_t cSq = c * c;
    scalar_t dSq = d * d;
    dcm(0, 0) = aSq + bSq - cSq - dSq;
    dcm(0, 1) = 2 * (b * c - a * d);
    dcm(0, 2) = 2 * (a * c + b * d);
    dcm(1, 0) = 2 * (b * c + a * d);
    dcm(1, 1) = aSq - bSq + cSq - dSq;
    dcm(1, 2) = 2 * (c * d - a * b);
    dcm(2, 0) = 2 * (b * d - a * c);
    dcm(2, 1) = 2 * (a * b + c * d);
    dcm(2, 2) = aSq - bSq - cSq + dSq;
    return dcm;
  }

  /**
   * Constructor from euler angles
   *
   * This sets the instance to a quaternion representing coordinate transformation from
   * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
   * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
   *
   * @param euler euler angle instance
   */
  inline quat from_euler(const vec3& euler) {
    quat q;
    scalar_t cosPhi_2 = cos(euler(0) / 2.0);
    scalar_t cosTheta_2 = cos(euler(1) / 2.0);
    scalar_t cosPsi_2 = cos(euler(2) / 2.0);
    scalar_t sinPhi_2 = sin(euler(0) / 2.0);
    scalar_t sinTheta_2 = sin(euler(1) / 2.0);
    scalar_t sinPsi_2 = sin(euler(2) / 2.0);
    q.w() = cosPhi_2 * cosTheta_2 * cosPsi_2 +
           sinPhi_2 * sinTheta_2 * sinPsi_2;
    q.x() = sinPhi_2 * cosTheta_2 * cosPsi_2 -
           cosPhi_2 * sinTheta_2 * sinPsi_2;
    q.y() = cosPhi_2 * sinTheta_2 * cosPsi_2 +
           sinPhi_2 * cosTheta_2 * sinPsi_2;
    q.z() = cosPhi_2 * cosTheta_2 * sinPsi_2 -
           sinPhi_2 * sinTheta_2 * cosPsi_2;
    return q;
  }

  /**
    * Constructor from dcm
    *
    * Instance is initialized from a dcm representing coordinate transformation
    * from frame 2 to frame 1.
    *
    * @param dcm dcm to set quaternion to
    */
  inline quat dcm2quat(const mat3& R) {
    quat q;
    scalar_t t = R.trace();
    if (t > scalar_t(0)) {
      t = sqrt(scalar_t(1) + t);
      q.w() = scalar_t(0.5) * t;
      t = scalar_t(0.5) / t;
      q.x() = (R(2,1) - R(1,2)) * t;
      q.y() = (R(0,2) - R(2,0)) * t;
      q.z() = (R(1,0) - R(0,1)) * t;
    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
      t = sqrt(scalar_t(1) + R(0,0) - R(1,1) - R(2,2));
      q.x() = scalar_t(0.5) * t;
      t = scalar_t(0.5) / t;
      q.w() = (R(2,1) - R(1,2)) * t;
      q.y() = (R(1,0) + R(0,1)) * t;
      q.z() = (R(0,2) + R(2,0)) * t;
    } else if (R(1,1) > R(2,2)) {
      t = sqrt(scalar_t(1) - R(0,0) + R(1,1) - R(2,2));
      q.y() = scalar_t(0.5) * t;
      t = scalar_t(0.5) / t;
      q.w() = (R(0,2) - R(2,0)) * t;
      q.x() = (R(1,0) + R(0,1)) * t;
      q.z() = (R(2,1) + R(1,2)) * t;
    } else {
      t = sqrt(scalar_t(1) - R(0,0) - R(1,1) + R(2,2));
      q.z() = scalar_t(0.5) * t;
      t = scalar_t(0.5) / t;
      q.w() = (R(1,0) - R(0,1)) * t;
      q.x() = (R(0,2) + R(2,0)) * t;
      q.y() = (R(2,1) + R(1,2)) * t;
    }
    return q;
  }
  
  inline vec2 constrainXY(const vec2 &v0, const vec2 &v1, const scalar_t &max) {
    if (vec2(v0 + v1).norm() <= max) {
      // vector does not exceed maximum magnitude
      return v0 + v1;

    } else if (v0.norm() >= max) {
      // the magnitude along v0, which has priority, already exceeds maximum.
      return v0.normalized() * max;

    } else if (fabsf(vec2(v1 - v0).norm()) < 0.001f) {
      // the two vectors are equal
      return v0.normalized() * max;

    } else if (v0.norm() < 0.001f) {
      // the first vector is 0.
      return v1.normalized() * max;

    } else {
      // vf = final vector with ||vf|| <= max
      // s = scaling factor
      // u1 = unit of v1
      // vf = v0 + v1 = v0 + s * u1
      // constraint: ||vf|| <= max
      //
      // solve for s: ||vf|| = ||v0 + s * u1|| <= max
      //
      // Derivation:
      // For simplicity, replace v0 -> v, u1 -> u
      // 				   		   v0(0/1/2) -> v0/1/2
      // 				   		   u1(0/1/2) -> u0/1/2
      //
      // ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
      // v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
      // s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
      //
      // quadratic equation:
      // -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
      //
      // b = 2 * u.dot(v)
      // a = 1 (because u is normalized)
      // c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
      //
      // sqrt(b^2 - 4*a*c) =
      // 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
      //
      // s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
      //      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
      // m = u.dot(v)
      // s = -m + sqrt(m^2 - c)
      //
      //
      //
      // notes:
      // 	- s (=scaling factor) needs to be positive
      // 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement

      vec2 u1 = v1.normalized();
      scalar_t m = u1.dot(v0);
      scalar_t c = v0.dot(v0) - max * max;
      scalar_t s = -m + sqrtf(m * m - c);
      return v0 + u1 * s;
    }
  }

  inline mavros_msgs::AttitudeTarget thrustToAttitude(const vec3 &thr_sp, const scalar_t yaw_sp) {
    mavros_msgs::AttitudeTarget att_sp = {};
    att_sp.type_mask = 1 + 2;

    // desired body_z axis = -normalize(thrust_vector)
    vec3 body_x, body_y, body_z;

    if (thr_sp.norm() > 0.00001f) {
      body_z = -thr_sp.normalized();

    } else {
      // no thrust, set Z axis to safe value
      body_z.fill(0);
      body_z(2) = 1.0f;
    }

    // vector of desired yaw direction in XY plane, rotated by PI/2
    vec3 y_C(-sinf(yaw_sp), cosf(yaw_sp), 0.0f);

    if (fabsf(body_z(2)) > 0.000001f) {
      // desired body_x axis, orthogonal to body_z
      body_x = y_C.cross(body_z);

      // keep nose to front while inverted upside down
      if (body_z(2) < 0.0f) {
        body_x = -body_x;
      }

      body_x.normalize();

    } else {
      // desired thrust is in XY plane, set X downside to construct correct matrix,
      // but yaw component will not be used actually
      body_x.fill(0);
      body_x(2) = 1.0f;
    }

    // desired body_y axis
    body_y = body_z.cross(body_x);

    mat3 R_sp;

    // fill rotation matrix
    for (int i = 0; i < 3; i++) {
      R_sp(i, 0) = body_x(i);
      R_sp(i, 1) = body_y(i);
      R_sp(i, 2) = body_z(i);
    }

    //copy quaternion setpoint to attitude setpoint topic
    quat q_sp = dcm2quat(R_sp);
    //convert from frd orientation wrt ned to flu orientation wrt enu
    q_sp = q_ng.conjugate() * q_sp * q_rb.conjugate();
    tf::quaternionEigenToMsg(q_sp, att_sp.orientation);
    //convert thrust from frd orientation wrt ned to flu orientation wrt enu
    att_sp.thrust = thr_sp.norm();

    return att_sp;
  }
}

#endif