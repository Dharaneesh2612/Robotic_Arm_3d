import 'dart:math';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:robot_arm_simulator/kinematics/transformations.dart';
import 'package:robot_arm_simulator/kinematics/forward_kinematics.dart';
import 'package:robot_arm_simulator/kinematics/inverse_kinematics.dart';

void main() {
  group('3-DOF Kinematics Tests', () {
    final fkEngine = ForwardKinematicsEngine();
    final ikEngine = InverseKinematicsEngine();

    test('Forward Kinematics Home Position (0, 0, 0)', () {
      final res = fkEngine.compute(0, 0, 0);
      // At theta1=0, theta2=0, theta3=0:
      // Link1 is horizontal along X (L1 = 2.5)
      // Link2 is horizontal along X (L2 = 2.0)
      // Base height = 0.5
      // Expected X = 2.5 + 2.0 = 4.5, Y = 0.0, Z = 0.5
      expect(res.position.x, closeTo(4.5, 0.001));
      expect(res.position.y, closeTo(0.0, 0.001));
      expect(res.position.z, closeTo(0.5, 0.001));
    });

    test('FK and IK Round-trip consistency', () {
      final double theta1 = 30.0;
      final double theta2 = 45.0;
      final double theta3 = -30.0;

      final fkRes = fkEngine.compute(theta1, theta2, theta3);
      final targetPos = fkRes.position;

      final ikRes = ikEngine.solve(targetPos.x, targetPos.y, targetPos.z);
      expect(ikRes.reachable, isTrue);
      expect(ikRes.solutions.isNotEmpty, isTrue);

      // Verify that at least one solution produces the target position
      bool matched = false;
      for (final sol in ikRes.solutions) {
        final reFK = fkEngine.compute(sol.theta1, sol.theta2, sol.theta3);
        if ((reFK.position.x - targetPos.x).abs() < 0.01 &&
            (reFK.position.y - targetPos.y).abs() < 0.01 &&
            (reFK.position.z - targetPos.z).abs() < 0.01) {
          matched = true;
          break;
        }
      }
      expect(matched, isTrue);
    });

    test('Workspace reachability checking', () {
      // Point outside maximum reach (R_max = 4.5)
      final outRes = ikEngine.solve(10.0, 10.0, 10.0);
      expect(outRes.reachable, isFalse);

      // Point inside reach
      final inRes = ikEngine.solve(2.0, 1.0, 1.0);
      expect(inRes.reachable, isTrue);
    });
  });
}
