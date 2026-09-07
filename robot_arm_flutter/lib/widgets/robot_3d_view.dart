import 'dart:math';
import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart' show Vector3;
import '../robot/robot_model.dart';

/// The custom 3D painter that draws the robotic arm in 2D canvas.
///
/// Implements a simple isometric-like projection with perspective:
///   screenX = (worldX - worldY) * cos(30°) * scale + centerX
///   screenY = (worldX + worldY) * sin(30°) * scale - worldZ * scale + centerY
///
/// This gives a clear 3D impression without needing a full 3D engine.
class Robot3DPainter extends CustomPainter {
  final RobotState state;
  final Offset? targetPoint;
  final double scale;
  final double rotationAngle; // additional view rotation

  const Robot3DPainter({
    required this.state,
    this.targetPoint,
    this.scale = 55.0,
    this.rotationAngle = 0.0,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width * 0.5, size.height * 0.65);

    // ── Draw ground grid ──────────────────────────────────────────────────
    _drawGrid(canvas, center);

    // ── Draw coordinate axes ──────────────────────────────────────────────
    _drawAxes(canvas, center);

    // ── Draw target point ─────────────────────────────────────────────────
    if (targetPoint != null) {
      _drawTargetPoint(canvas, center, targetPoint!);
    }

    // ── Draw robot arm ────────────────────────────────────────────────────
    final joints = state.jointPositions;
    if (joints.length >= 4) {
      final base      = joints[0];
      final shoulder  = joints[1];
      final elbow     = joints[2];
      final wrist     = joints[3];

      _drawRobotArm(canvas, center, base, shoulder, elbow, wrist);
    }
  }

  // ── Projection ─────────────────────────────────────────────────────────
  Offset _project(Vector3 p, Offset center) {
    final angle = rotationAngle;
    // Rotate around Z axis for view rotation
    final rx = p.x * cos(angle) - p.y * sin(angle);
    final ry = p.x * sin(angle) + p.y * cos(angle);

    // Isometric-like projection
    final sx = (rx - ry) * cos(30.0 * pi / 180.0) * scale + center.dx;
    final sy = (rx + ry) * sin(30.0 * pi / 180.0) * scale - p.z * scale + center.dy;
    return Offset(sx, sy);
  }

  // ── Ground Grid ────────────────────────────────────────────────────────
  void _drawGrid(Canvas canvas, Offset center) {
    final paint = Paint()
      ..color = Colors.white.withValues(alpha: 0.08)
      ..strokeWidth = 0.5
      ..style = PaintingStyle.stroke;

    const gridSize = 5;
    const step = 1.0;

    for (double i = -gridSize; i <= gridSize; i += step) {
      final p1 = _project(Vector3(i, -gridSize.toDouble(), 0), center);
      final p2 = _project(Vector3(i,  gridSize.toDouble(), 0), center);
      canvas.drawLine(p1, p2, paint);

      final p3 = _project(Vector3(-gridSize.toDouble(), i, 0), center);
      final p4 = _project(Vector3( gridSize.toDouble(), i, 0), center);
      canvas.drawLine(p3, p4, paint);
    }
  }

  // ── Coordinate Axes ───────────────────────────────────────────────────
  void _drawAxes(Canvas canvas, Offset center) {
    final origin = _project(Vector3.zero(), center);
    final xEnd   = _project(Vector3(1.2, 0, 0), center);
    final yEnd   = _project(Vector3(0, 1.2, 0), center);
    final zEnd   = _project(Vector3(0, 0, 1.2), center);

    _drawAxisLine(canvas, origin, xEnd, Colors.red,   'X');
    _drawAxisLine(canvas, origin, yEnd, Colors.green, 'Y');
    _drawAxisLine(canvas, origin, zEnd, Colors.blue,  'Z');
  }

  void _drawAxisLine(Canvas canvas, Offset from, Offset to, Color color, String label) {
    canvas.drawLine(from, to, Paint()..color = color..strokeWidth = 2.0);
    final tp = TextPainter(
      text: TextSpan(text: label, style: TextStyle(color: color, fontSize: 10)),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, to + const Offset(3, -5));
  }

  // ── Target Point ───────────────────────────────────────────────────────
  void _drawTargetPoint(Canvas canvas, Offset center, Offset target3d) {
    // target3d is actually (x,y,z) packed into Offset for convenience
    // But we receive it as a Vector3 via the targetPoint Offset
    // We'll handle this properly by receiving Vector3 separately.
  }

  // ── Robot Arm ─────────────────────────────────────────────────────────
  void _drawRobotArm(
    Canvas canvas,
    Offset center,
    Vector3 base,
    Vector3 shoulder,
    Vector3 elbow,
    Vector3 wrist,
  ) {
    final pBase     = _project(base,     center);
    final pShoulder = _project(shoulder, center);
    final pElbow    = _project(elbow,    center);
    final pWrist    = _project(wrist,    center);

    // ── Base cylinder (drawn as rectangle) ──
    _drawBaseBlock(canvas, pBase, pShoulder);

    // ── Upper arm segment ──
    _drawSegment(canvas, pShoulder, pElbow,
      const Color(0xFF1E88E5), const Color(0xFF42A5F5), width: 8.0);

    // ── Lower arm segment ──
    _drawSegment(canvas, pElbow, pWrist,
      const Color(0xFF00897B), const Color(0xFF26A69A), width: 6.5);

    // ── Joints ──
    _drawJoint(canvas, pShoulder, 10.0, const Color(0xFFE53935)); // Shoulder
    _drawJoint(canvas, pElbow,     8.0, const Color(0xFFFB8C00)); // Elbow
    _drawJoint(canvas, pWrist,     7.0, const Color(0xFFAB47BC)); // Wrist

    // ── End effector/gripper ──
    _drawGripper(canvas, pWrist);
  }

  void _drawBaseBlock(Canvas canvas, Offset bottom, Offset top) {
    // Draw base as a thick vertical line
    canvas.drawLine(
      bottom, top,
      Paint()
        ..color = const Color(0xFF455A64)
        ..strokeWidth = 18.0
        ..strokeCap = StrokeCap.round,
    );
    // Base plate
    canvas.drawCircle(
      bottom, 14.0,
      Paint()..color = const Color(0xFF37474F),
    );
  }

  void _drawSegment(Canvas canvas, Offset from, Offset to,
      Color colorDark, Color colorLight, {double width = 7.0}) {
    // Shadow
    canvas.drawLine(
      from + const Offset(2, 2), to + const Offset(2, 2),
      Paint()
        ..color = Colors.black.withValues(alpha: 0.3)
        ..strokeWidth = width + 2
        ..strokeCap = StrokeCap.round,
    );
    // Main segment
    canvas.drawLine(
      from, to,
      Paint()
        ..color = colorDark
        ..strokeWidth = width
        ..strokeCap = StrokeCap.round,
    );
    // Highlight
    canvas.drawLine(
      from, to,
      Paint()
        ..color = colorLight.withValues(alpha: 0.5)
        ..strokeWidth = width * 0.3
        ..strokeCap = StrokeCap.round,
    );
  }

  void _drawJoint(Canvas canvas, Offset pos, double radius, Color color) {
    canvas.drawCircle(pos, radius + 2, Paint()..color = Colors.black38);
    canvas.drawCircle(pos, radius, Paint()..color = color);
    canvas.drawCircle(
      pos, radius * 0.45,
      Paint()..color = Colors.white.withValues(alpha: 0.5),
    );
  }

  void _drawGripper(Canvas canvas, Offset pos) {
    // Draw a small diamond shape for the end effector
    final path = Path()
      ..moveTo(pos.dx, pos.dy - 10)
      ..lineTo(pos.dx + 6, pos.dy)
      ..lineTo(pos.dx, pos.dy + 10)
      ..lineTo(pos.dx - 6, pos.dy)
      ..close();
    canvas.drawPath(path, Paint()..color = const Color(0xFFEC407A));
    canvas.drawPath(
      path,
      Paint()
        ..color = Colors.white.withValues(alpha: 0.3)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5,
    );
  }

  @override
  bool shouldRepaint(Robot3DPainter oldDelegate) =>
      oldDelegate.state != state ||
      oldDelegate.rotationAngle != rotationAngle;
}

/// Widget that wraps the custom painter and supports view rotation.
class Robot3DView extends StatefulWidget {
  final RobotState state;
  final Vector3? targetPos;

  const Robot3DView({super.key, required this.state, this.targetPos});

  @override
  State<Robot3DView> createState() => _Robot3DViewState();
}

class _Robot3DViewState extends State<Robot3DView> {
  double _rotation = 0.0;
  double _startRotation = 0.0;
  double _panStartDx = 0.0;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onHorizontalDragStart: (details) {
        _startRotation = _rotation;
        _panStartDx = details.localPosition.dx;
      },
      onHorizontalDragUpdate: (details) {
        setState(() {
          _rotation = _startRotation +
              (details.localPosition.dx - _panStartDx) * 0.01;
        });
      },
      child: CustomPaint(
        painter: Robot3DPainter(
          state: widget.state,
          scale: 55.0,
          rotationAngle: _rotation,
        ),
        child: _buildTargetOverlay(),
      ),
    );
  }

  Widget _buildTargetOverlay() {
    return const SizedBox.expand();
  }
}
