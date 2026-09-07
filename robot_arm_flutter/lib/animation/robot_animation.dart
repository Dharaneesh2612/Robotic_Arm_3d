import 'dart:async';
import 'dart:math';
import '../robot/robot_model.dart';

/// Smooth joint interpolation animator for the robot arm.
///
/// Uses smoothstep (ease-in/ease-out) interpolation:
///   f(t) = 3t² − 2t³  where t ∈ [0, 1]
///
/// The animator drives from [fromState] to [toState] over [durationMs] ms.
class RobotAnimator {
  final RobotState fromState;
  final RobotState toState;
  final int durationMs;
  final void Function(RobotState) onUpdate;
  final void Function() onComplete;

  Timer? _timer;
  DateTime? _startTime;
  bool _cancelled = false;

  RobotAnimator({
    required this.fromState,
    required this.toState,
    required this.durationMs,
    required this.onUpdate,
    required this.onComplete,
  });

  /// Start the animation timer (~60fps).
  void start() {
    _startTime = DateTime.now();
    _cancelled = false;

    // Tick every ~16ms (~60fps)
    _timer = Timer.periodic(const Duration(milliseconds: 16), _tick);
  }

  /// Cancel a running animation.
  void cancel() {
    _cancelled = true;
    _timer?.cancel();
    _timer = null;
  }

  void _tick(Timer timer) {
    if (_cancelled) {
      timer.cancel();
      return;
    }

    final elapsed = DateTime.now().difference(_startTime!).inMilliseconds;
    final t = (elapsed / durationMs).clamp(0.0, 1.0);

    // Smoothstep function: eases in and out
    final smooth = t * t * (3.0 - 2.0 * t);

    // Interpolate each joint angle linearly in joint space
    final t1 = _lerp(fromState.theta1Deg, toState.theta1Deg, smooth);
    final t2 = _lerp(fromState.theta2Deg, toState.theta2Deg, smooth);
    final t3 = _lerp(fromState.theta3Deg, toState.theta3Deg, smooth);

    final interpolatedState = RobotModel.computeState(t1, t2, t3);
    onUpdate(interpolatedState);

    if (t >= 1.0) {
      timer.cancel();
      onComplete();
    }
  }

  static double _lerp(double a, double b, double t) => a + (b - a) * t;
}
