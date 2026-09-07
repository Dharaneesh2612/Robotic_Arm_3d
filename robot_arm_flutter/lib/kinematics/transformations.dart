import 'dart:math';
import 'package:vector_math/vector_math_64.dart';

/// Denavit-Hartenberg (DH) transformation matrix.
///
/// Parameters:
///   [a]     : Link length (distance along x-axis between z-axes)
///   [alpha] : Link twist (angle between z-axes, about x-axis)
///   [d]     : Link offset (distance along z-axis)
///   [theta] : Joint angle (angle about z-axis), in radians
///
/// Returns a 4x4 homogeneous transformation matrix.
Matrix4 dhTransform(double a, double alpha, double d, double theta) {
  final ct = cos(theta);
  final st = sin(theta);
  final ca = cos(alpha);
  final sa = sin(alpha);

  // Column-major storage (Flutter/Dart Matrix4 is column-major)
  return Matrix4(
    ct,       st,       0.0,  0.0,   // column 0
    -st * ca, ct * ca,  sa,   0.0,   // column 1
    st * sa,  -ct * sa, ca,   0.0,   // column 2
    a * ct,   a * st,   d,    1.0,   // column 3 (translation)
  );
}

/// Forward Kinematics for the 3-DOF articulated arm.
///
/// DH Parameters used:
///   Joint 1 (base rotation): a=0,  alpha=pi/2, d=BASE_H, theta=theta1
///   Joint 2 (shoulder):      a=L1, alpha=0,    d=0,      theta=theta2
///   Joint 3 (elbow):         a=L2, alpha=0,    d=0,      theta=theta3
///
/// The result is T = T1 * T2 * T3.
class TransformationMatrices {
  static const double L1 = 2.5;       // Upper arm length
  static const double L2 = 2.0;       // Lower arm length
  static const double baseH = 0.5;    // Base height

  /// Returns the 4x4 end-effector transformation matrix.
  static Matrix4 computeFK(double theta1, double theta2, double theta3) {
    // Base rotation (z-axis), raises along Z by baseH, twists x by pi/2
    final t1 = dhTransform(0.0, pi / 2, baseH, theta1);
    // Shoulder rotation along the horizontal arm segment L1
    final t2 = dhTransform(L1, 0.0, 0.0, theta2);
    // Elbow rotation along the lower arm segment L2
    final t3 = dhTransform(L2, 0.0, 0.0, theta3);

    // Chain the transforms: world -> base -> shoulder -> elbow -> end-effector
    return t1 * t2 * t3;
  }

  /// Extract the XYZ position from a 4x4 homogeneous transform matrix.
  static Vector3 extractPosition(Matrix4 t) {
    return Vector3(t[12], t[13], t[14]);
  }

  /// Extract Euler XYZ angles (in degrees) from the 3x3 rotation sub-matrix.
  ///
  /// Returns [rx, ry, rz] in degrees.
  static Vector3 extractEulerDeg(Matrix4 t) {
    // Rotation matrix elements (column-major Matrix4)
    final r00 = t[0];
    final r10 = t[1];
    final r20 = t[2];
    final r21 = t[6];
    final r22 = t[10];

    final sy = sqrt(r00 * r00 + r10 * r10);
    double rx, ry, rz;

    if (sy > 1e-6) {
      rx = atan2(r21, r22);
      ry = atan2(-r20, sy);
      rz = atan2(r10, r00);
    } else {
      rx = atan2(-t[9], t[5]);
      ry = atan2(-r20, sy);
      rz = 0.0;
    }

    return Vector3(
      rx * 180.0 / pi,
      ry * 180.0 / pi,
      rz * 180.0 / pi,
    );
  }

  /// Compute the shoulder, elbow, and wrist joint positions in world coordinates.
  ///
  /// Returns a list of 4 points: [base, shoulder, elbow, wrist/end-effector]
  static List<Vector3> computeJointPositions(
    double theta1, double theta2, double theta3,
  ) {
    final t1 = theta1;
    final t2 = theta2;
    final t3 = theta3;

    // Base joint is at the origin (ground)
    final base = Vector3(0.0, 0.0, 0.0);

    // Shoulder is at the top of the base cylinder
    final shoulder = Vector3(0.0, 0.0, baseH);

    // Elbow = shoulder + L1 along the arm direction (theta1 in XY plane, theta2 elevation)
    final elbow = Vector3(
      shoulder.x + L1 * cos(t2) * cos(t1),
      shoulder.y + L1 * cos(t2) * sin(t1),
      shoulder.z + L1 * sin(t2),
    );

    // Wrist/end-effector = elbow + L2
    final wrist = Vector3(
      elbow.x + L2 * cos(t2 + t3) * cos(t1),
      elbow.y + L2 * cos(t2 + t3) * sin(t1),
      elbow.z + L2 * sin(t2 + t3),
    );

    return [base, shoulder, elbow, wrist];
  }
}
