import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart';
import '../kinematics/inverse_kinematics.dart';
import '../animation/robot_animation.dart';
import 'robot_model.dart';

/// Operating mode of the robot controller.
enum ControlMode { fk, ik }

/// Result of a single target in the agent task sequence.
class SequenceTargetResult {
  final Vector3 target;
  final bool success;
  final String message;

  const SequenceTargetResult({
    required this.target,
    required this.success,
    required this.message,
  });
}

/// Central controller (ChangeNotifier) for the Robot Arm simulator.
///
/// Manages:
///   - Current robot state (joint angles, FK pose)
///   - FK slider changes
///   - IK solves
///   - Animation playback
///   - Agent task sequence execution
class RobotController extends ChangeNotifier {
  // ── State ────────────────────────────────────────────────────────────────
  RobotState _state = RobotState.initial();
  ControlMode _mode = ControlMode.fk;
  String _statusMessage = 'Ready';
  bool _isAnimating = false;

  // ── Animation ────────────────────────────────────────────────────────────
  RobotAnimator? _animator;

  // ── Target Sequence ──────────────────────────────────────────────────────
  List<SequenceTargetResult> _sequenceResults = [];
  int _currentSequenceIndex = -1;

  // ── Getters ──────────────────────────────────────────────────────────────
  RobotState get state => _state;
  ControlMode get mode => _mode;
  String get statusMessage => _statusMessage;
  bool get isAnimating => _isAnimating;
  List<SequenceTargetResult> get sequenceResults => _sequenceResults;
  int get currentSequenceIndex => _currentSequenceIndex;

  double get theta1Deg => _state.theta1Deg;
  double get theta2Deg => _state.theta2Deg;
  double get theta3Deg => _state.theta3Deg;

  // ── Mode ─────────────────────────────────────────────────────────────────
  void setMode(ControlMode mode) {
    _mode = mode;
    notifyListeners();
  }

  // ── FK Controls ──────────────────────────────────────────────────────────
  void setTheta1(double deg) => _setJointAngles(deg, _state.theta2Deg, _state.theta3Deg);
  void setTheta2(double deg) => _setJointAngles(_state.theta1Deg, deg, _state.theta3Deg);
  void setTheta3(double deg) => _setJointAngles(_state.theta1Deg, _state.theta2Deg, deg);

  void _setJointAngles(double t1, double t2, double t3) {
    _state = RobotModel.computeState(t1, t2, t3);
    _statusMessage = 'FK: θ1=${t1.toStringAsFixed(1)}°  θ2=${t2.toStringAsFixed(1)}°  θ3=${t3.toStringAsFixed(1)}°';
    notifyListeners();
  }

  // ── IK Solve ─────────────────────────────────────────────────────────────
  /// Solve IK for target [x, y, z] and animate to the solution.
  void solveIK(double x, double y, double z) {
    if (_isAnimating) return;

    final (newState, result) = RobotModel.solveIK(x, y, z);
    if (!result.success) {
      _statusMessage = 'IK failed: ${result.message}';
      notifyListeners();
      return;
    }

    _statusMessage = 'IK solved → Target: (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)}, ${z.toStringAsFixed(2)})';
    _animateTo(newState!);
  }

  // ── Reset ─────────────────────────────────────────────────────────────────
  void reset() {
    _cancelAnimation();
    _state = RobotState.initial();
    _statusMessage = 'Reset to home position';
    _sequenceResults = [];
    _currentSequenceIndex = -1;
    notifyListeners();
  }

  // ── Agent Task Sequence ──────────────────────────────────────────────────
  /// The default agent task targets.
  static const List<List<double>> defaultTargets = [
    [2.5, 0.0, 2.0],
    [2.0, 1.5, 2.2],
    [1.0, 2.0, 1.8],
    [0.0, 2.5, 2.0],
    [-1.5, 1.5, 2.2],
    [-2.0, 0.0, 2.0],
    [0.0, 0.0, 2.5],
  ];

  /// Run the built-in target sequence.
  Future<void> runDefaultTargetSequence() async {
    final targets = defaultTargets
        .map((t) => Vector3(t[0], t[1], t[2]))
        .toList();
    await runTargetSequence(targets);
  }

  /// Execute a sequence of Cartesian targets with smooth animation.
  ///
  /// For each target:
  ///   1. Check reachability (workspace check)
  ///   2. Solve IK (joint angles)
  ///   3. Animate smoothly to the new configuration
  ///   4. Record result (success/failure)
  Future<void> runTargetSequence(List<Vector3> targets) async {
    if (_isAnimating) return;

    _sequenceResults = [];
    _currentSequenceIndex = -1;
    _isAnimating = true;
    _statusMessage = 'Running target sequence...';
    notifyListeners();

    for (int i = 0; i < targets.length; i++) {
      _currentSequenceIndex = i;
      final target = targets[i];
      _statusMessage = 'Target ${i + 1}/${targets.length}: '
          '(${target.x.toStringAsFixed(2)}, ${target.y.toStringAsFixed(2)}, ${target.z.toStringAsFixed(2)})';
      notifyListeners();

      final (newState, result) = RobotModel.solveIK(target.x, target.y, target.z);

      if (!result.success) {
        _sequenceResults.add(SequenceTargetResult(
          target: target,
          success: false,
          message: result.message,
        ));
        notifyListeners();
        await Future.delayed(const Duration(milliseconds: 400));
        continue;
      }

      // Smoothly animate to the new joint configuration
      await _animateToAsync(newState!);

      _sequenceResults.add(SequenceTargetResult(
        target: target,
        success: true,
        message: 'Reached successfully',
      ));
      notifyListeners();

      // Short pause between targets
      await Future.delayed(const Duration(milliseconds: 300));
    }

    _isAnimating = false;
    _currentSequenceIndex = -1;
    _statusMessage = 'Sequence complete. '
        '${_sequenceResults.where((r) => r.success).length}/${targets.length} targets reached.';
    notifyListeners();
  }

  // ── Internal Animation Helpers ────────────────────────────────────────────
  void _animateTo(RobotState targetState) {
    _cancelAnimation();
    _isAnimating = true;

    _animator = RobotAnimator(
      fromState: _state,
      toState: targetState,
      durationMs: 800,
      onUpdate: (state) {
        _state = state;
        notifyListeners();
      },
      onComplete: () {
        _state = targetState;
        _isAnimating = false;
        notifyListeners();
      },
    )..start();
  }

  Future<void> _animateToAsync(RobotState targetState) {
    final completer = _CancelableCompleter();
    _cancelAnimation();
    _isAnimating = true;

    _animator = RobotAnimator(
      fromState: _state,
      toState: targetState,
      durationMs: 1000,
      onUpdate: (state) {
        _state = state;
        notifyListeners();
      },
      onComplete: () {
        _state = targetState;
        _isAnimating = false;
        notifyListeners();
        completer.complete();
      },
    )..start();

    return completer.future;
  }

  void _cancelAnimation() {
    _animator?.cancel();
    _animator = null;
    _isAnimating = false;
  }

  @override
  void dispose() {
    _cancelAnimation();
    super.dispose();
  }
}

/// Simple completer helper for async animation awaiting.
class _CancelableCompleter {
  bool _completed = false;
  late final _future = Future<void>(() async {
    while (!_completed) {
      await Future.delayed(const Duration(milliseconds: 16));
    }
  });

  Future<void> get future => _future;
  void complete() => _completed = true;
}
