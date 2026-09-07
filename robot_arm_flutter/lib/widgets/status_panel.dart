import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../robot/robot_controller.dart';

/// Panel showing the current robot state: joint angles, EE position, status.
class StatusPanel extends StatelessWidget {
  const StatusPanel({super.key});

  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<RobotController>();
    final state = ctrl.state;
    final pos = state.endEffectorPos;

    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.06),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.white.withValues(alpha: 0.08)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // ── Status Message ─────────────────────────────────────────────
          Row(
            children: [
              Icon(
                ctrl.isAnimating ? Icons.sync : Icons.info_outline,
                size: 16,
                color: ctrl.isAnimating
                    ? const Color(0xFF1E88E5)
                    : Colors.white54,
              ),
              const SizedBox(width: 6),
              Expanded(
                child: Text(
                  ctrl.statusMessage,
                  style: TextStyle(
                    color: ctrl.isAnimating
                        ? const Color(0xFF42A5F5)
                        : Colors.white70,
                    fontSize: 12,
                  ),
                  overflow: TextOverflow.ellipsis,
                  maxLines: 2,
                ),
              ),
            ],
          ),
          const Divider(color: Colors.white12, height: 16),

          // ── Joint Angles ───────────────────────────────────────────────
          _label('Joint Angles'),
          const SizedBox(height: 4),
          Row(children: [
            _chip('θ₁', '${state.theta1Deg.toStringAsFixed(1)}°',
                const Color(0xFF1E88E5)),
            const SizedBox(width: 6),
            _chip('θ₂', '${state.theta2Deg.toStringAsFixed(1)}°',
                const Color(0xFF00897B)),
            const SizedBox(width: 6),
            _chip('θ₃', '${state.theta3Deg.toStringAsFixed(1)}°',
                const Color(0xFFAB47BC)),
          ]),
          const SizedBox(height: 10),

          // ── End Effector Position ──────────────────────────────────────
          _label('End-Effector Position'),
          const SizedBox(height: 4),
          Row(children: [
            _coord('X', pos.x.toStringAsFixed(3), Colors.redAccent),
            const SizedBox(width: 6),
            _coord('Y', pos.y.toStringAsFixed(3), Colors.greenAccent),
            const SizedBox(width: 6),
            _coord('Z', pos.z.toStringAsFixed(3), Colors.blueAccent),
          ]),
          const SizedBox(height: 10),

          // ── Orientation ───────────────────────────────────────────────
          _label('EE Orientation (Euler, deg)'),
          const SizedBox(height: 4),
          Row(children: [
            _coord('Rx', state.endEffectorOriDeg.x.toStringAsFixed(1),
                Colors.redAccent),
            const SizedBox(width: 6),
            _coord('Ry', state.endEffectorOriDeg.y.toStringAsFixed(1),
                Colors.greenAccent),
            const SizedBox(width: 6),
            _coord('Rz', state.endEffectorOriDeg.z.toStringAsFixed(1),
                Colors.blueAccent),
          ]),
        ],
      ),
    );
  }

  Widget _label(String text) => Text(
        text,
        style: const TextStyle(
            color: Colors.white38, fontSize: 11, letterSpacing: 0.5),
      );

  Widget _chip(String label, String value, Color color) => Expanded(
        child: Container(
          padding: const EdgeInsets.symmetric(vertical: 5, horizontal: 6),
          decoration: BoxDecoration(
            color: color.withValues(alpha: 0.12),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: color.withValues(alpha: 0.3)),
          ),
          child: Column(
            children: [
              Text(label,
                  style: TextStyle(color: color, fontSize: 10, fontWeight: FontWeight.bold)),
              Text(value,
                  style: const TextStyle(color: Colors.white, fontSize: 12)),
            ],
          ),
        ),
      );

  Widget _coord(String label, String value, Color color) => Expanded(
        child: RichText(
          text: TextSpan(children: [
            TextSpan(
                text: '$label: ',
                style: TextStyle(color: color, fontSize: 11, fontWeight: FontWeight.bold)),
            TextSpan(
                text: value,
                style: const TextStyle(color: Colors.white, fontSize: 11)),
          ]),
        ),
      );
}
