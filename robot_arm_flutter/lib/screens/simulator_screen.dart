import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../robot/robot_controller.dart';
import '../widgets/robot_3d_view.dart';
import '../widgets/joint_controls.dart';
import '../widgets/ik_controls.dart';
import '../widgets/status_panel.dart';

/// The main simulator screen with 3D view, FK/IK controls, and status panel.
class SimulatorScreen extends StatelessWidget {
  const SimulatorScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF0A0E1A),
      body: SafeArea(
        child: Column(
          children: [
            // ── Header ─────────────────────────────────────────────────────
            _SimHeader(),

            // ── 3D View ────────────────────────────────────────────────────
            const Expanded(
              flex: 5,
              child: _RobotViewSection(),
            ),

            // ── Bottom Controls Panel ──────────────────────────────────────
            const Expanded(
              flex: 6,
              child: _ControlPanel(),
            ),
          ],
        ),
      ),
    );
  }
}

// ── Header ────────────────────────────────────────────────────────────────────
class _SimHeader extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            const Color(0xFF1A237E).withValues(alpha: 0.9),
            const Color(0xFF0A0E1A),
          ],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        border: const Border(
          bottom: BorderSide(color: Color(0xFF1E88E5), width: 0.5),
        ),
      ),
      child: Row(
        children: [
          IconButton(
            icon: const Icon(Icons.arrow_back_ios_new, color: Colors.white70, size: 20),
            onPressed: () => Navigator.pop(context),
            padding: EdgeInsets.zero,
          ),
          const SizedBox(width: 8),
          const Expanded(
            child: Text(
              '3-DOF ROBOTIC ARM',
              style: TextStyle(
                color: Colors.white,
                fontSize: 16,
                fontWeight: FontWeight.bold,
                letterSpacing: 1.5,
              ),
            ),
          ),
          // Mode Toggle
          _ModeToggle(),
        ],
      ),
    );
  }
}

class _ModeToggle extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<RobotController>();
    return Container(
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.08),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          _ToggleBtn(
            label: 'FK',
            active: ctrl.mode == ControlMode.fk,
            onTap: () => ctrl.setMode(ControlMode.fk),
          ),
          _ToggleBtn(
            label: 'IK',
            active: ctrl.mode == ControlMode.ik,
            onTap: () => ctrl.setMode(ControlMode.ik),
          ),
        ],
      ),
    );
  }
}

class _ToggleBtn extends StatelessWidget {
  final String label;
  final bool active;
  final VoidCallback onTap;

  const _ToggleBtn(
      {required this.label, required this.active, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 6),
        decoration: BoxDecoration(
          color: active ? const Color(0xFF1E88E5) : Colors.transparent,
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          label,
          style: TextStyle(
            color: active ? Colors.white : Colors.white54,
            fontWeight: FontWeight.bold,
            fontSize: 13,
          ),
        ),
      ),
    );
  }
}

// ── 3D View Section ───────────────────────────────────────────────────────────
class _RobotViewSection extends StatelessWidget {
  const _RobotViewSection();

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotController>().state;

    return Container(
      margin: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: const Color(0xFF0D1117),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.white.withValues(alpha: 0.07)),
        boxShadow: [
          BoxShadow(
            color: const Color(0xFF1E88E5).withValues(alpha: 0.08),
            blurRadius: 20,
          ),
        ],
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(16),
        child: Robot3DView(state: state),
      ),
    );
  }
}

// ── Control Panel ─────────────────────────────────────────────────────────────
class _ControlPanel extends StatelessWidget {
  const _ControlPanel();

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFF0D1B2A),
        borderRadius: const BorderRadius.vertical(top: Radius.circular(20)),
        border: Border(
          top: BorderSide(color: Colors.white.withValues(alpha: 0.08)),
        ),
      ),
      child: SingleChildScrollView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            // Controls (FK or IK)
            _ControlsSection(),
            const SizedBox(height: 12),

            // Status Panel
            const StatusPanel(),
            const SizedBox(height: 12),

            // Action buttons
            _ActionButtons(),
          ],
        ),
      ),
    );
  }
}

class _ControlsSection extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final mode = context.watch<RobotController>().mode;
    return AnimatedSwitcher(
      duration: const Duration(milliseconds: 250),
      child: mode == ControlMode.fk
          ? const JointControls(key: ValueKey('fk'))
          : const IKControls(key: ValueKey('ik')),
    );
  }
}

class _ActionButtons extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<RobotController>();

    return Row(
      children: [
        // Reset
        Expanded(
          child: OutlinedButton.icon(
            onPressed: ctrl.isAnimating ? null : ctrl.reset,
            icon: const Icon(Icons.refresh, size: 18),
            label: const Text('Reset'),
            style: OutlinedButton.styleFrom(
              foregroundColor: Colors.white70,
              side: BorderSide(color: Colors.white.withValues(alpha: 0.2)),
              padding: const EdgeInsets.symmetric(vertical: 12),
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10)),
            ),
          ),
        ),
        const SizedBox(width: 10),

        // Run Target Sequence
        Expanded(
          flex: 2,
          child: ElevatedButton.icon(
            onPressed: ctrl.isAnimating
                ? null
                : () {
                    ctrl.runDefaultTargetSequence();
                    _showSequenceSheet(context);
                  },
            icon: ctrl.isAnimating
                ? const SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(
                        strokeWidth: 2, color: Colors.white),
                  )
                : const Icon(Icons.play_arrow, size: 18),
            label: Text(ctrl.isAnimating ? 'Running...' : 'Run Sequence'),
            style: ElevatedButton.styleFrom(
              backgroundColor: const Color(0xFF6A1B9A),
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

  void _showSequenceSheet(BuildContext context) {
    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: Colors.transparent,
      builder: (_) => ChangeNotifierProvider.value(
        value: context.read<RobotController>(),
        child: const _SequenceResultSheet(),
      ),
    );
  }
}

// ── Sequence Result Sheet ─────────────────────────────────────────────────────
class _SequenceResultSheet extends StatelessWidget {
  const _SequenceResultSheet();

  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<RobotController>();
    final results = ctrl.sequenceResults;
    final total = RobotController.defaultTargets.length;

    return Container(
      height: MediaQuery.of(context).size.height * 0.6,
      decoration: const BoxDecoration(
        color: Color(0xFF0D1B2A),
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Handle
          Center(
            child: Container(
              width: 40,
              height: 4,
              decoration: BoxDecoration(
                color: Colors.white24,
                borderRadius: BorderRadius.circular(2),
              ),
            ),
          ),
          const SizedBox(height: 12),

          // Title + progress
          Row(
            children: [
              const Text(
                'AGENT TASK SEQUENCE',
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 14,
                  letterSpacing: 1,
                ),
              ),
              const Spacer(),
              if (ctrl.isAnimating)
                const SizedBox(
                  width: 16,
                  height: 16,
                  child: CircularProgressIndicator(
                      strokeWidth: 2, color: Color(0xFF1E88E5)),
                ),
            ],
          ),

          // Progress bar
          const SizedBox(height: 8),
          LinearProgressIndicator(
            value: total == 0 ? 0 : results.length / total,
            backgroundColor: Colors.white12,
            valueColor: const AlwaysStoppedAnimation(Color(0xFF1E88E5)),
            borderRadius: BorderRadius.circular(4),
          ),
          const SizedBox(height: 4),
          Text(
            '${results.length} / $total targets processed',
            style: const TextStyle(color: Colors.white38, fontSize: 11),
          ),
          const SizedBox(height: 12),

          // Target list
          Expanded(
            child: ListView.separated(
              physics: const BouncingScrollPhysics(),
              itemCount: total,
              separatorBuilder: (_, __) =>
                  const Divider(color: Colors.white10, height: 8),
              itemBuilder: (_, i) {
                final t = RobotController.defaultTargets[i];
                final tStr =
                    '(${t[0]}, ${t[1]}, ${t[2]})';
                final isCurrent = ctrl.currentSequenceIndex == i;
                final isDone = i < results.length;
                final isSuccess = isDone && results[i].success;

                return AnimatedContainer(
                  duration: const Duration(milliseconds: 300),
                  padding: const EdgeInsets.symmetric(
                      horizontal: 12, vertical: 8),
                  decoration: BoxDecoration(
                    color: isCurrent
                        ? const Color(0xFF1E88E5).withValues(alpha: 0.15)
                        : Colors.transparent,
                    borderRadius: BorderRadius.circular(8),
                    border: isCurrent
                        ? Border.all(
                            color: const Color(0xFF1E88E5).withValues(alpha: 0.4))
                        : null,
                  ),
                  child: Row(
                    children: [
                      // Status icon
                      Icon(
                        isCurrent
                            ? Icons.radio_button_checked
                            : isDone
                                ? (isSuccess
                                    ? Icons.check_circle
                                    : Icons.cancel)
                                : Icons.radio_button_unchecked,
                        size: 18,
                        color: isCurrent
                            ? const Color(0xFF1E88E5)
                            : isDone
                                ? (isSuccess ? Colors.greenAccent : Colors.redAccent)
                                : Colors.white24,
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              'Target ${i + 1}: $tStr',
                              style: TextStyle(
                                color: isCurrent
                                    ? Colors.white
                                    : isDone
                                        ? Colors.white70
                                        : Colors.white38,
                                fontSize: 13,
                                fontWeight: isCurrent
                                    ? FontWeight.bold
                                    : FontWeight.normal,
                              ),
                            ),
                            if (isDone)
                              Text(
                                results[i].message,
                                style: TextStyle(
                                  color: isSuccess
                                      ? Colors.greenAccent.withValues(alpha: 0.7)
                                      : Colors.redAccent.withValues(alpha: 0.7),
                                  fontSize: 11,
                                ),
                              ),
                          ],
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
