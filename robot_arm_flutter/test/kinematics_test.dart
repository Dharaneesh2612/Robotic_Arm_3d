import 'dart:math';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:robot_arm_simulator/kinematics/transformations.dart';
import 'package:robot_arm_simulator/kinematics/forward_kinematics.dart';
import 'package:robot_arm_simulator/kinematics/inverse_kinematics.dart';

void main() {
  group('3-DOF Kinematics Tests', () {
    test('Forward Kinematics Home Position (0, 0, 0)', () {
      final res = ForwardKinematics.computeFromDeg(0, 0, 0);
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

      final fkRes = ForwardKinematics.computeFromDeg(theta1, theta2, theta3);
      final targetPos = fkRes.position;

      final ikRes = InverseKinematics.solve(targetPos.x, targetPos.y, targetPos.z);
      expect(ikRes.success, isTrue);
      expect(ikRes.jointAnglesDeg, isNotNull);

      final solDeg = ikRes.jointAnglesDeg!;
      final reFK = ForwardKinematics.computeFromDeg(solDeg.x, solDeg.y, solDeg.z);
      expect(reFK.position.x, closeTo(targetPos.x, 0.05));
      expect(reFK.position.y, closeTo(targetPos.y, 0.05));
      expect(reFK.position.z, closeTo(targetPos.z, 0.05));
    });

    test('Workspace reachability checking', () {
      // Point outside maximum reach (R_max = 4.5)
      final outRes = InverseKinematics.solve(10.0, 10.0, 10.0);
      expect(outRes.success, isFalse);

      // Point inside reach
      final inRes = InverseKinematics.solve(2.0, 1.0, 1.0);
      expect(inRes.success, isTrue);
    });
  });
}
