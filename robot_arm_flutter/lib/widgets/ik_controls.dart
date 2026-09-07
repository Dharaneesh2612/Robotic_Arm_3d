import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../robot/robot_controller.dart';

/// IK Cartesian input controls: X, Y, Z fields + Solve button.
class IKControls extends StatefulWidget {
  const IKControls({super.key});

  @override
  State<IKControls> createState() => _IKControlsState();
}

class _IKControlsState extends State<IKControls> {
  final _xCtrl = TextEditingController(text: '2.5');
  final _yCtrl = TextEditingController(text: '0.0');
  final _zCtrl = TextEditingController(text: '2.0');

  @override
  void dispose() {
    _xCtrl.dispose();
    _yCtrl.dispose();
    _zCtrl.dispose();
    super.dispose();
  }

  void _solveIK(BuildContext context) {
    final x = double.tryParse(_xCtrl.text.trim());
    final y = double.tryParse(_yCtrl.text.trim());
    final z = double.tryParse(_zCtrl.text.trim());

    if (x == null || y == null || z == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Please enter valid numbers for X, Y, Z')),
      );
      return;
    }

    context.read<RobotController>().solveIK(x, y, z);
  }

  @override
  Widget build(BuildContext context) {
    final isAnimating = context.watch<RobotController>().isAnimating;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text(
          'Cartesian Target (IK)',
          style: TextStyle(
            color: Colors.white70,
            fontSize: 13,
            fontWeight: FontWeight.w600,
            letterSpacing: 0.8,
          ),
        ),
        const SizedBox(height: 10),
        Row(
          children: [
            _CoordField(label: 'X', controller: _xCtrl),
            const SizedBox(width: 8),
            _CoordField(label: 'Y', controller: _yCtrl),
            const SizedBox(width: 8),
            _CoordField(label: 'Z', controller: _zCtrl),
          ],
        ),
        const SizedBox(height: 10),
        SizedBox(
          width: double.infinity,
          child: ElevatedButton.icon(
            onPressed: isAnimating ? null : () => _solveIK(context),
            icon: const Icon(Icons.calculate_outlined, size: 18),
            label: const Text('Solve IK'),
            style: ElevatedButton.styleFrom(
              backgroundColor: const Color(0xFF1E88E5),
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(vertical: 12),
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10)),
              disabledBackgroundColor: Colors.white12,
            ),
          ),
        ),
      ],
    );
  }
}

class _CoordField extends StatelessWidget {
  final String label;
  final TextEditingController controller;

  const _CoordField({required this.label, required this.controller});

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(label,
              style: const TextStyle(color: Colors.white54, fontSize: 11)),
          const SizedBox(height: 4),
          TextField(
            controller: controller,
            keyboardType: const TextInputType.numberWithOptions(
                decimal: true, signed: true),
            style: const TextStyle(color: Colors.white, fontSize: 14),
            decoration: InputDecoration(
              filled: true,
              fillColor: Colors.white.withValues(alpha: 0.07),
              contentPadding:
                  const EdgeInsets.symmetric(horizontal: 10, vertical: 10),
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: BorderSide.none,
              ),
              focusedBorder: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: const BorderSide(color: Color(0xFF1E88E5)),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
