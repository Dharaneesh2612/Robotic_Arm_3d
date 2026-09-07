import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../robot/robot_controller.dart';
import 'simulator_screen.dart';

/// Home / landing screen with app info and launch button.
class HomeScreen extends StatelessWidget {
  const HomeScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF060B14),
      body: Stack(
        children: [
          // Background gradient orbs
          _BackgroundOrbs(),

          SafeArea(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 24),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const SizedBox(height: 48),

                  // ── Logo / Header ────────────────────────────────────────
                  _AppHeader(),
                  const SizedBox(height: 40),

                  // ── Feature Cards ────────────────────────────────────────
                  const Text(
                    'FEATURES',
                    style: TextStyle(
                      color: Colors.white38,
                      fontSize: 11,
                      letterSpacing: 2,
                    ),
                  ),
                  const SizedBox(height: 12),
                  _FeatureCard(
                    icon: Icons.rotate_90_degrees_cw_outlined,
                    color: const Color(0xFF1E88E5),
                    title: 'Forward Kinematics',
                    desc: 'Direct joint angle control with DH matrices. '
                        'Real-time end-effector pose computation.',
                  ),
                  const SizedBox(height: 10),
                  _FeatureCard(
                    icon: Icons.my_location,
                    color: const Color(0xFF00897B),
                    title: 'Inverse Kinematics',
                    desc: 'Analytical IK solver. Enter Cartesian (X, Y, Z) '
                        'and get joint angles instantly.',
                  ),
                  const SizedBox(height: 10),
                  _FeatureCard(
                    icon: Icons.route,
                    color: const Color(0xFF6A1B9A),
                    title: 'Agent Task Sequence',
                    desc: 'Automated multi-target trajectory execution with '
                        'smooth animation and reachability checking.',
                  ),

                  const Spacer(),

                  // ── Robot Specs ───────────────────────────────────────────
                  _SpecsRow(),
                  const SizedBox(height: 24),

                  // ── Launch Button ─────────────────────────────────────────
                  _LaunchButton(),
                  const SizedBox(height: 32),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _BackgroundOrbs extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned(
          top: -80,
          right: -60,
          child: Container(
            width: 260,
            height: 260,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              gradient: RadialGradient(
                colors: [
                  const Color(0xFF1E88E5).withValues(alpha: 0.18),
                  Colors.transparent,
                ],
              ),
            ),
          ),
        ),
        Positioned(
          bottom: 60,
          left: -80,
          child: Container(
            width: 220,
            height: 220,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              gradient: RadialGradient(
                colors: [
                  const Color(0xFF6A1B9A).withValues(alpha: 0.15),
                  Colors.transparent,
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }
}

class _AppHeader extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Container(
          width: 56,
          height: 56,
          decoration: BoxDecoration(
            gradient: const LinearGradient(
              colors: [Color(0xFF1E88E5), Color(0xFF6A1B9A)],
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
            ),
            borderRadius: BorderRadius.circular(16),
          ),
          child: const Icon(Icons.precision_manufacturing_outlined,
              color: Colors.white, size: 30),
        ),
        const SizedBox(height: 20),
        const Text(
          '3-DOF Robotic\nArm Simulator',
          style: TextStyle(
            color: Colors.white,
            fontSize: 32,
            fontWeight: FontWeight.bold,
            height: 1.2,
          ),
        ),
        const SizedBox(height: 8),
        const Text(
          'Interactive FK • IK • Trajectory Simulation',
          style: TextStyle(color: Colors.white54, fontSize: 14),
        ),
      ],
    );
  }
}

class _FeatureCard extends StatelessWidget {
  final IconData icon;
  final Color color;
  final String title;
  final String desc;

  const _FeatureCard({
    required this.icon,
    required this.color,
    required this.title,
    required this.desc,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.04),
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: color.withValues(alpha: 0.2)),
      ),
      child: Row(
        children: [
          Container(
            width: 40,
            height: 40,
            decoration: BoxDecoration(
              color: color.withValues(alpha: 0.12),
              borderRadius: BorderRadius.circular(10),
            ),
            child: Icon(icon, color: color, size: 22),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(title,
                    style: const TextStyle(
                        color: Colors.white,
                        fontSize: 14,
                        fontWeight: FontWeight.w600)),
                const SizedBox(height: 2),
                Text(desc,
                    style: const TextStyle(color: Colors.white54, fontSize: 12),
                    maxLines: 2),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _SpecsRow extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 16),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.04),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          _spec('L1', '2.5 u', const Color(0xFF1E88E5)),
          _vDivider(),
          _spec('L2', '2.0 u', const Color(0xFF00897B)),
          _vDivider(),
          _spec('DOF', '3', const Color(0xFF6A1B9A)),
          _vDivider(),
          _spec('Base', '0.5 u', Colors.orange),
        ],
      ),
    );
  }

  Widget _spec(String label, String val, Color color) => Column(
        children: [
          Text(label,
              style: const TextStyle(color: Colors.white38, fontSize: 11)),
          Text(val,
              style: TextStyle(
                  color: color,
                  fontSize: 14,
                  fontWeight: FontWeight.bold)),
        ],
      );

  Widget _vDivider() => Container(
      width: 1, height: 30, color: Colors.white.withValues(alpha: 0.08));
}

class _LaunchButton extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: double.infinity,
      child: ElevatedButton(
        onPressed: () {
          Navigator.push(
            context,
            MaterialPageRoute(
              builder: (_) => ChangeNotifierProvider(
                create: (_) => RobotController(),
                child: const SimulatorScreen(),
              ),
            ),
          );
        },
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          padding: EdgeInsets.zero,
          shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(14)),
        ).copyWith(
          backgroundColor: WidgetStateProperty.all(Colors.transparent),
          elevation: WidgetStateProperty.all(0),
          overlayColor: WidgetStateProperty.all(Colors.white10),
        ),
        child: Ink(
          decoration: BoxDecoration(
            gradient: const LinearGradient(
              colors: [Color(0xFF1E88E5), Color(0xFF6A1B9A)],
              begin: Alignment.centerLeft,
              end: Alignment.centerRight,
            ),
            borderRadius: BorderRadius.circular(14),
          ),
          child: Container(
            padding: const EdgeInsets.symmetric(vertical: 16),
            alignment: Alignment.center,
            child: const Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(Icons.play_circle_fill, color: Colors.white, size: 22),
                SizedBox(width: 10),
                Text(
                  'LAUNCH SIMULATOR',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 15,
                    fontWeight: FontWeight.bold,
                    letterSpacing: 1.5,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
