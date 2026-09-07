import 'package:vector_math/vector_math_64.dart';
import '../kinematics/forward_kinematics.dart';
import '../kinematics/inverse_kinematics.dart';
import '../kinematics/transformations.dart';

/// Snapshot of the robot's complete state.
class RobotState {
  /// Current joint angles in degrees.
  final double theta1Deg;
  final double theta2Deg;
  final double theta3Deg;

  /// Computed end-effector position [x, y, z].
  final Vector3 endEffectorPos;

  /// Computed end-effector orientation [rx, ry, rz] in degrees.
  final Vector3 endEffectorOriDeg;

  /// Joint world positions: [base, shoulder, elbow, wrist]
  final List<Vector3> jointPositions;

  const RobotState({
    required this.theta1Deg,
    required this.theta2Deg,
    required this.theta3Deg,
    required this.endEffectorPos,
    required this.endEffectorOriDeg,
    required this.jointPositions,
  });

  /// Default starting state.
  factory RobotState.initial() {
    final fk = ForwardKinematics.computeFromDeg(0.0, 20.0, 30.0);
    return RobotState(
      theta1Deg: 0.0,
      theta2Deg: 20.0,
      theta3Deg: 30.0,
      endEffectorPos: fk.position,
      endEffectorOriDeg: fk.orientationDeg,
      jointPositions: fk.jointPositions,
    );
  }
}

/// Core robot model: holds parameters and FK/IK solver access.
class RobotModel {
  /// Arm link lengths (world units).
  static const double L1 = TransformationMatrices.L1;
  static const double L2 = TransformationMatrices.L2;
  static const double baseHeight = TransformationMatrices.baseH;

  /// Joint limits (degrees).
  static const double joint1Min = kJoint1Min;
  static const double joint1Max = kJoint1Max;
  static const double joint2Min = kJoint2Min;
  static const double joint2Max = kJoint2Max;
  static const double joint3Min = kJoint3Min;
  static const double joint3Max = kJoint3Max;

  /// Compute the robot state for given joint angles (degrees).
  static RobotState computeState(
    double theta1Deg,
    double theta2Deg,
    double theta3Deg,
  ) {
    final fk = ForwardKinematics.computeFromDeg(theta1Deg, theta2Deg, theta3Deg);
    return RobotState(
      theta1Deg: theta1Deg,
      theta2Deg: theta2Deg,
      theta3Deg: theta3Deg,
      endEffectorPos: fk.position,
      endEffectorOriDeg: fk.orientationDeg,
      jointPositions: fk.jointPositions,
    );
  }

  /// Attempt to solve IK and return the resulting RobotState.
  /// Returns null if IK fails.
  static (RobotState?, IKResult) solveIK(double x, double y, double z) {
    final result = InverseKinematics.solve(x, y, z);
    if (!result.success) return (null, result);

    final angles = result.jointAnglesDeg!;
    final state = computeState(angles.x, angles.y, angles.z);
    return (state, result);
  }
}
