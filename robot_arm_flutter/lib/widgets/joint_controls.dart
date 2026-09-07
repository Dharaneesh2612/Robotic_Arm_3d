import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../robot/robot_controller.dart';
import '../kinematics/inverse_kinematics.dart';

/// Slider-based controls for Forward Kinematics mode.
class JointControls extends StatelessWidget {
  const JointControls({super.key});

  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<RobotController>();

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _sectionHeader('Joint Angles (FK)'),
        const SizedBox(height: 8),
        _JointSlider(
          label: 'θ₁  (Base)',
          value: ctrl.theta1Deg,
          min: kJoint1Min,
          max: kJoint1Max,
          color: const Color(0xFF1E88E5),
          onChanged: (v) => ctrl.setTheta1(v),
        ),
        _JointSlider(
          label: 'θ₂  (Shoulder)',
          value: ctrl.theta2Deg,
          min: kJoint2Min,
          max: kJoint2Max,
          color: const Color(0xFF00897B),
          onChanged: (v) => ctrl.setTheta2(v),
        ),
        _JointSlider(
          label: 'θ₃  (Elbow)',
          value: ctrl.theta3Deg,
          min: kJoint3Min,
          max: kJoint3Max,
          color: const Color(0xFFAB47BC),
          onChanged: (v) => ctrl.setTheta3(v),
        ),
      ],
    );
  }

  Widget _sectionHeader(String title) => Text(
    title,
    style: const TextStyle(
      color: Colors.white70,
      fontSize: 13,
      fontWeight: FontWeight.w600,
      letterSpacing: 0.8,
    ),
  );
}

class _JointSlider extends StatelessWidget {
  final String label;
  final double value;
  final double min;
  final double max;
  final Color color;
  final void Function(double) onChanged;

  const _JointSlider({
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.color,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 4),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(label,
                  style: const TextStyle(color: Colors.white60, fontSize: 12)),
              Text('${value.toStringAsFixed(1)}°',
                  style: TextStyle(
                      color: color,
                      fontSize: 12,
                      fontWeight: FontWeight.bold)),
            ],
          ),
          SliderTheme(
            data: SliderThemeData(
              activeTrackColor: color,
              thumbColor: color,
              inactiveTrackColor: color.withValues(alpha: 0.2),
              overlayColor: color.withValues(alpha: 0.15),
              trackHeight: 3.0,
              thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 7),
            ),
            child: Slider(
              value: value.clamp(min, max),
              min: min,
              max: max,
              onChanged: onChanged,
            ),
          ),
        ],
      ),
    );
  }
}
