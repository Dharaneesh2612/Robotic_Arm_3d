import 'dart:math';
import 'package:vector_math/vector_math_64.dart';
import 'transformations.dart';

/// Result of a Forward Kinematics calculation.
class FKResult {
  /// End-effector position in world coordinates [x, y, z].
  final Vector3 position;

  /// End-effector orientation as Euler angles [rx, ry, rz] in degrees.
  final Vector3 orientationDeg;

  /// Joint positions: [base, shoulder, elbow, wrist]
  final List<Vector3> jointPositions;

  /// Full 4x4 end-effector transformation matrix.
  final Matrix4 transformMatrix;

  const FKResult({
    required this.position,
    required this.orientationDeg,
    required this.jointPositions,
    required this.transformMatrix,
  });
}

/// Forward Kinematics Engine for the 3-DOF articulated arm.
///
/// Input:  joint angles [theta1, theta2, theta3] in radians.
/// Output: [FKResult] containing the end-effector pose and joint positions.
class ForwardKinematics {
  static const double L1 = TransformationMatrices.L1;
  static const double L2 = TransformationMatrices.L2;
  static const double baseH = TransformationMatrices.baseH;

  /// Compute full FK for the 3-DOF arm.
  ///
  /// [theta1] : Base rotation (radians)
  /// [theta2] : Shoulder elevation (radians)
  /// [theta3] : Elbow angle (radians)
  static FKResult compute(double theta1, double theta2, double theta3) {
    // Compute end-effector transform via DH chain
    final T = TransformationMatrices.computeFK(theta1, theta2, theta3);

    // Extract position and orientation
    final position = TransformationMatrices.extractPosition(T);
    final orientation = TransformationMatrices.extractEulerDeg(T);

    // Compute intermediate joint positions for 3D rendering
    final joints = TransformationMatrices.computeJointPositions(
      theta1, theta2, theta3,
    );

    return FKResult(
      position: position,
      orientationDeg: orientation,
      jointPositions: joints,
      transformMatrix: T,
    );
  }

  /// Convenience: compute FK from degrees instead of radians.
  static FKResult computeFromDeg(
    double theta1Deg, double theta2Deg, double theta3Deg,
  ) {
    return compute(
      theta1Deg * pi / 180.0,
      theta2Deg * pi / 180.0,
      theta3Deg * pi / 180.0,
    );
  }
}
