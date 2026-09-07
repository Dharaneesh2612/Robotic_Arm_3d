import 'dart:math';
import 'package:vector_math/vector_math_64.dart';
import 'transformations.dart';

/// Joint limit constraints (in degrees).
const double kJoint1Min = -180.0;
const double kJoint1Max =  180.0;
const double kJoint2Min =  -90.0;
const double kJoint2Max =   90.0;
const double kJoint3Min = -150.0;
const double kJoint3Max =  150.0;

/// Result of an Inverse Kinematics solve.
class IKResult {
  /// Whether a valid solution was found.
  final bool success;

  /// Solved joint angles in degrees [theta1, theta2, theta3].
  /// null when [success] is false.
  final Vector3? jointAnglesDeg;

  /// Solved joint angles in radians [theta1, theta2, theta3].
  /// null when [success] is false.
  final Vector3? jointAnglesRad;

  /// Human-readable message describing success or the reason for failure.
  final String message;

  const IKResult.success({
    required Vector3 anglesDeg,
    required Vector3 anglesRad,
  })  : success = true,
        jointAnglesDeg = anglesDeg,
        jointAnglesRad = anglesRad,
        message = 'IK solved successfully';

  const IKResult.failure(this.message)
      : success = false,
        jointAnglesDeg = null,
        jointAnglesRad = null;
}

/// Analytical Inverse Kinematics for the 3-DOF articulated arm.
///
/// This arm has exactly 3 DOF, meaning IK can solve for position (x, y, z)
/// only — orientation is determined by the resulting joint angles.
///
/// Robot parameters:
///   L1 = 2.5  (upper arm length)
///   L2 = 2.0  (lower arm length)
///   baseH = 0.5  (base height offset)
///
/// Solution approach:
///   theta1 = atan2(y, x)              (base rotation to face the target)
///
///   r  = sqrt(x² + y²)               (horizontal reach)
///   z' = z - baseH                    (height relative to shoulder)
///
///   D = (r² + z'² - L1² - L2²) / (2·L1·L2)   (cosine rule)
///
///   theta3 = atan2(sqrt(1 - D²), D)   (elbow-down solution)
///   theta2 = atan2(z', r) - atan2(L2·sin(theta3), L1 + L2·cos(theta3))
///
/// Workspace check: |D| <= 1.0  (geometrically reachable)
/// Joint limits: applied after solving.
class InverseKinematics {
  static const double L1 = TransformationMatrices.L1;
  static const double L2 = TransformationMatrices.L2;
  static const double baseH = TransformationMatrices.baseH;

  /// Maximum reachable radius from the base shoulder point.
  static const double maxReach = L1 + L2;

  /// Minimum reachable radius (fully folded arm).
  static const double minReach = (L1 - L2) < 0 ? 0.0 : L1 - L2;

  /// Solve IK for a Cartesian target [x, y, z].
  ///
  /// Returns [IKResult.success] with joint angles in degrees and radians,
  /// or [IKResult.failure] with a descriptive error message.
  static IKResult solve(double x, double y, double z) {
    // ── Step 1: Workspace check ────────────────────────────────────────────
    // Height relative to the shoulder (shoulder is at z = baseH)
    final double zRel = z - baseH;

    // Horizontal distance from base axis
    final double r = sqrt(x * x + y * y);

    // Total distance from shoulder to target
    final double distSq = r * r + zRel * zRel;
    final double dist = sqrt(distSq);

    if (dist > maxReach + 1e-6) {
      return IKResult.failure(
        'Target is OUT OF REACH. Distance from shoulder: '
        '${dist.toStringAsFixed(3)}, Max reach: ${maxReach.toStringAsFixed(3)}.',
      );
    }
    if (dist < minReach - 1e-6 && minReach > 0) {
      return IKResult.failure(
        'Target is TOO CLOSE (inside minimum reach). '
        'Min reach: ${minReach.toStringAsFixed(3)}.',
      );
    }

    // ── Step 2: Cosine rule for theta3 ────────────────────────────────────
    // D = cos(theta3) from the law of cosines
    final double D = (distSq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    // Clamp numerically to handle floating-point edge cases
    final double DClamped = D.clamp(-1.0, 1.0);

    // Elbow-down configuration: theta3 > 0
    final double theta3 = atan2(sqrt(1.0 - DClamped * DClamped), DClamped);

    // ── Step 3: Compute theta2 ────────────────────────────────────────────
    // atan2(z', r) gives the angle to the target in the vertical plane
    // Subtract the arm geometry correction term
    final double theta2 = atan2(zRel, r) -
        atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));

    // ── Step 4: Compute theta1 ────────────────────────────────────────────
    // Rotation in the horizontal XY plane to face the target
    final double theta1 = atan2(y, x);

    // ── Step 5: Convert to degrees ────────────────────────────────────────
    final double t1Deg = theta1 * 180.0 / pi;
    final double t2Deg = theta2 * 180.0 / pi;
    final double t3Deg = theta3 * 180.0 / pi;

    // ── Step 6: Joint limit check ─────────────────────────────────────────
    if (t1Deg < kJoint1Min || t1Deg > kJoint1Max) {
      return IKResult.failure(
        'Theta1 (${t1Deg.toStringAsFixed(1)}°) exceeds joint limits '
        '[$kJoint1Min°, $kJoint1Max°].',
      );
    }
    if (t2Deg < kJoint2Min || t2Deg > kJoint2Max) {
      return IKResult.failure(
        'Theta2 (${t2Deg.toStringAsFixed(1)}°) exceeds joint limits '
        '[$kJoint2Min°, $kJoint2Max°].',
      );
    }
    if (t3Deg < kJoint3Min || t3Deg > kJoint3Max) {
      return IKResult.failure(
        'Theta3 (${t3Deg.toStringAsFixed(1)}°) exceeds joint limits '
        '[$kJoint3Min°, $kJoint3Max°].',
      );
    }

    return IKResult.success(
      anglesDeg: Vector3(t1Deg, t2Deg, t3Deg),
      anglesRad: Vector3(theta1, theta2, theta3),
    );
  }

  /// Quick workspace-only reachability check (no joint-limit check).
  static bool isReachable(double x, double y, double z) {
    final zRel = z - baseH;
    final r = sqrt(x * x + y * y);
    final dist = sqrt(r * r + zRel * zRel);
    return dist <= maxReach + 1e-6;
  }
}
