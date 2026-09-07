import open3d as o3d
import numpy as np
import math
import time


# ============================================================
# 3-DOF ROBOT PARAMETERS
# ============================================================

L1 = 2.5       # Upper arm length
L2 = 2.0       # Lower arm length
BASE_H = 0.5   # Base height

JOINT_MIN = [-180, -90, -150]
JOINT_MAX = [180, 90, 150]


# ============================================================
# KINEMATICS
# ============================================================

def dh_transform(a, alpha, d, theta):
    """
    Denavit-Hartenberg transformation matrix.
    Angles are in radians.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,        d],
        [0,        0,        0,         1]
    ])


def forward_kinematics(q):
    """
    FK for a simple 3-DOF articulated arm.

    q = [theta1, theta2, theta3] in degrees

    Returns:
        4x4 end-effector transformation matrix
    """

    t1, t2, t3 = np.radians(q)

    # Base rotation
    T1 = dh_transform(0, np.pi / 2, BASE_H, t1)

    # Shoulder -> elbow
    T2 = dh_transform(L1, 0, 0, t2)

    # Elbow -> end effector
    T3 = dh_transform(L2, 0, 0, t3)

    return T1 @ T2 @ T3


def fk_position(q):
    """
    Return end-effector XYZ position.
    """

    T = forward_kinematics(q)
    return T[:3, 3]


# ============================================================
# ANALYTICAL IK
# ============================================================

def inverse_kinematics(x, y, z):
    """
    Analytical IK.

    The orientation target is not independently controllable
    because a 3-DOF arm has only 3 joint variables.

    Returns:
        [theta1, theta2, theta3]
        or None if unreachable.
    """

    # Remove base height
    z_rel = z - BASE_H

    # Joint 1
    theta1 = math.atan2(y, x)

    # Horizontal distance
    r = math.sqrt(x * x + y * y)

    # Distance from shoulder to target
    D = (r * r + z_rel * z_rel - L1**2 - L2**2) / (2 * L1 * L2)

    # Workspace check
    if D < -1.0 or D > 1.0:
        return None

    # Numerical safety
    D = np.clip(D, -1.0, 1.0)

    # Elbow-down solution
    theta3 = math.atan2(
        math.sqrt(1 - D**2),
        D
    )

    theta2 = math.atan2(z_rel, r) - math.atan2(
        L2 * math.sin(theta3),
        L1 + L2 * math.cos(theta3)
    )

    q = np.degrees([
        theta1,
        theta2,
        theta3
    ])

    # Joint limits
    for i in range(3):
        if q[i] < JOINT_MIN[i] or q[i] > JOINT_MAX[i]:
            return None

    return q


def pose_from_fk(q):
    """
    Convert FK matrix into:
    x, y, z, rx, ry, rz
    """

    T = forward_kinematics(q)

    position = T[:3, 3]
    R = T[:3, :3]

    # Euler XYZ extraction
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0

    return np.array([
        position[0],
        position[1],
        position[2],
        np.degrees(rx),
        np.degrees(ry),
        np.degrees(rz)
    ])


# ============================================================
# GEOMETRY HELPERS
# ============================================================

def create_box(size, center):
    mesh = o3d.geometry.TriangleMesh.create_box(
        width=size[0],
        height=size[1],
        depth=size[2]
    )

    mesh.translate(np.array(center) - np.array(size) / 2)

    mesh.compute_vertex_normals()

    return mesh


def create_cylinder_between(
        start,
        end,
        radius=0.15,
        color=None
):
    """
    Create a cylinder between two 3D points.
    """

    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

    direction = end - start
    length = np.linalg.norm(direction)

    if length < 1e-8:
        return o3d.geometry.TriangleMesh()

    cylinder = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius,
        height=length,
        resolution=32
    )

    cylinder.compute_vertex_normals()

    # Cylinder initially points along +Z
    z_axis = np.array([0, 0, 1.0])
    direction = direction / length

    v = np.cross(z_axis, direction)
    c = np.dot(z_axis, direction)

    if np.linalg.norm(v) < 1e-8:

        if c < 0:
            R = np.array([
                [1, 0, 0],
                [0, -1, 0],
                [0, 0, -1]
            ])
        else:
            R = np.eye(3)

    else:
        s = np.linalg.norm(v)

        vx = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

        R = (
            np.eye(3)
            + vx
            + vx @ vx * ((1 - c) / (s ** 2))
        )

    cylinder.rotate(R, center=(0, 0, 0))
    cylinder.translate((start + end) / 2)

    if color is not None:
        cylinder.paint_uniform_color(color)

    return cylinder


def create_sphere(position, radius=0.22, color=None):

    sphere = o3d.geometry.TriangleMesh.create_sphere(
        radius=radius,
        resolution=24
    )

    sphere.translate(position)
    sphere.compute_vertex_normals()

    if color is not None:
        sphere.paint_uniform_color(color)

    return sphere


# ============================================================
# ROBOT VISUALIZATION
# ============================================================

class RobotArm:

    def __init__(self):

        self.q = np.array([0.0, 20.0, 30.0])

        self.base = None
        self.upper_arm = None
        self.lower_arm = None
        self.joint1 = None
        self.joint2 = None
        self.gripper = None

        self.create_robot()


    def create_robot(self):

        # Base
        self.base = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.65,
            height=BASE_H,
            resolution=32
        )

        self.base.compute_vertex_normals()
        self.base.paint_uniform_color(
            [0.2, 0.2, 0.25]
        )


    def update_geometry(self):

        # Calculate joint positions
        t1, t2, t3 = np.radians(self.q)

        shoulder = np.array([
            0,
            0,
            BASE_H
        ])

        elbow = shoulder + np.array([
            L1 * np.cos(t2) * np.cos(t1),
            L1 * np.cos(t2) * np.sin(t1),
            L1 * np.sin(t2)
        ])

        wrist = elbow + np.array([
            L2 * np.cos(t2 + t3) * np.cos(t1),
            L2 * np.cos(t2 + t3) * np.sin(t1),
            L2 * np.sin(t2 + t3)
        ])

        # Upper arm
        self.upper_arm = create_cylinder_between(
            shoulder,
            elbow,
            radius=0.18,
            color=[0.1, 0.55, 0.9]
        )

        # Lower arm
        self.lower_arm = create_cylinder_between(
            elbow,
            wrist,
            radius=0.16,
            color=[0.15, 0.75, 0.45]
        )

        # Joints
        self.joint1 = create_sphere(
            shoulder,
            0.25,
            [0.9, 0.2, 0.2]
        )

        self.joint2 = create_sphere(
            elbow,
            0.23,
            [0.9, 0.7, 0.1]
        )

        # End effector
        self.gripper = create_sphere(
            wrist,
            0.20,
            [0.9, 0.3, 0.7]
        )

    def get_geometries(self):
        self.update_geometry()
        return [self.base, self.upper_arm, self.lower_arm, self.joint1, self.joint2, self.gripper]

    def update_robot(self):
        self.update_geometry()


# ============================================================

# OPEN3D GUI APPLICATION
# ============================================================

class RobotGUI:

    def __init__(self):

        self.robot = RobotArm()

        self.app = o3d.visualization.gui.Application.instance

        self.window = self.app.create_window(
            "3-DOF Robotic Arm - FK / IK Simulator",
            1400,
            850
        )

        self.scene = o3d.visualization.gui.SceneWidget()

        self.scene.scene = o3d.visualization.rendering.Open3DScene(
            self.window.renderer
        )

        self.scene.scene.set_background(
            [0.05, 0.05, 0.07, 1.0]
        )

        self.window.add_child(self.scene)

        self.panel = o3d.visualization.gui.Vert(
            12,
            o3d.visualization.gui.Margins(15, 15, 15, 15)
        )

        self.window.add_child(self.panel)

        self.create_controls()

        self.setup_scene()

        self.update_robot()

        self.window.set_on_layout(
            self.on_layout
        )

        self.animation_running = False
        self.animation_targets = []
        self.animation_index = 0
        self.animation_start = None
        self.animation_from = None
        self.animation_to = None


    # ========================================================
    # GUI CONTROLS
    # ========================================================

    def create_controls(self):

        title = o3d.visualization.gui.Label(
            "3-DOF ROBOT ARM"
        )

        self.panel.add_child(title)

        self.panel.add_fixed(
            10
        )

        # Mode
        self.mode = o3d.visualization.gui.Combobox()

        self.mode.add_item("FK - Joint Control")
        self.mode.add_item("IK - Cartesian Control")

        self.mode.selected_index = 0

        self.mode.set_on_selection_changed(
            self.mode_changed
        )

        self.panel.add_child(
            o3d.visualization.gui.Label("Mode")
        )

        self.panel.add_child(self.mode)

        self.panel.add_fixed(10)

        # Joint sliders
        self.sliders = []

        for i in range(3):

            label = o3d.visualization.gui.Label(
                f"Theta {i + 1}"
            )

            slider = o3d.visualization.gui.Slider(
                o3d.visualization.gui.Slider.DOUBLE
            )

            slider.set_limits(
                JOINT_MIN[i],
                JOINT_MAX[i]
            )

            slider.double_value = self.robot.q[i]

            slider.set_on_value_changed(
                self.slider_changed
            )

            self.panel.add_child(label)
            self.panel.add_child(slider)

            self.sliders.append(slider)

        self.panel.add_fixed(15)

        # IK inputs
        self.panel.add_child(
            o3d.visualization.gui.Label(
                "IK TARGET POSE"
            )
        )

        self.ik_fields = []

        names = [
            "X",
            "Y",
            "Z",
            "Rx",
            "Ry",
            "Rz"
        ]

        defaults = [
            "2.5",
            "0.0",
            "2.0",
            "0",
            "0",
            "0"
        ]

        for name, default in zip(names, defaults):

            field = o3d.visualization.gui.TextEdit()

            field.placeholder_text = name
            field.text_value = default

            self.panel.add_child(
                o3d.visualization.gui.Label(name)
            )

            self.panel.add_child(field)

            self.ik_fields.append(field)

        self.panel.add_fixed(10)

        # IK button
        self.ik_button = o3d.visualization.gui.Button(
            "Solve IK"
        )

        self.ik_button.set_on_clicked(
            self.solve_ik
        )

        self.panel.add_child(
            self.ik_button
        )

        # FK button
        self.fk_button = o3d.visualization.gui.Button(
            "Calculate FK"
        )

        self.fk_button.set_on_clicked(
            self.calculate_fk
        )

        self.panel.add_child(
            self.fk_button
        )

        self.panel.add_fixed(10)

        # Target animation
        self.panel.add_child(
            o3d.visualization.gui.Label(
                "AGENT TASK EXECUTION"
            )
        )

        self.animate_button = o3d.visualization.gui.Button(
            "Run Target Sequence"
        )

        self.animate_button.set_on_clicked(
            self.start_animation
        )

        self.panel.add_child(
            self.animate_button
        )

        self.status = o3d.visualization.gui.Label(
            "Ready"
        )

        self.panel.add_fixed(10)

        self.panel.add_child(
            self.status
        )


    # ========================================================
    # SCENE
    # ========================================================

    def setup_scene(self):

        # Coordinate frame
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0,
            origin=[0, 0, 0]
        )

        self.scene.scene.add_geometry(
            "frame",
            frame,
            o3d.visualization.rendering.MaterialRecord()
        )

        # Ground
        grid = o3d.geometry.LineSet()

        points = []
        lines = []

        size = 6
        step = 0.5

        index = 0

        for i in np.arange(-size, size + step, step):

            points.extend([
                [i, -size, 0],
                [i, size, 0],
                [-size, i, 0],
                [size, i, 0]
            ])

            lines.extend([
                [index, index + 1],
                [index + 2, index + 3]
            ])

            index += 4

        grid.points = o3d.utility.Vector3dVector(
            points
        )

        grid.lines = o3d.utility.Vector2iVector(
            lines
        )

        self.scene.scene.add_geometry(
            "grid",
            grid,
            o3d.visualization.rendering.MaterialRecord()
        )

        self.scene.scene.camera.look_at(
            [0, 0, 1.5],
            [5, 5, 4],
            [0, 0, 1]
        )


    # ========================================================
    # UPDATE ROBOT
    # ========================================================

    def update_robot(self):

        self.robot.update_geometry()

        geometries = {
            "base": self.robot.base,
            "upper": self.robot.upper_arm,
            "lower": self.robot.lower_arm,
            "joint1": self.robot.joint1,
            "joint2": self.robot.joint2,
            "gripper": self.robot.gripper
        }

        for name, geometry in geometries.items():

            material = (
                o3d.visualization.rendering.MaterialRecord()
            )

            material.shader = "defaultLit"

            self.scene.scene.remove_geometry(name)

            self.scene.scene.add_geometry(
                name,
                geometry,
                material
            )

        pose = pose_from_fk(
            self.robot.q
        )

        self.status.text = (
            f"XYZ = "
            f"({pose[0]:.2f}, "
            f"{pose[1]:.2f}, "
            f"{pose[2]:.2f})"
        )


    # ========================================================
    # FK SLIDER
    # ========================================================

    def slider_changed(self, value):

        if self.mode.selected_index != 0:
            return

        for i in range(3):
            self.robot.q[i] = (
                self.sliders[i].double_value
            )

        self.update_robot()


    # ========================================================
    # MODE
    # ========================================================

    def mode_changed(self, value, text):

        if value == 0:

            self.status.text = (
                "FK Mode: Adjust joint angles"
            )

        else:

            self.status.text = (
                "IK Mode: Enter target position"
            )


    # ========================================================
    # FK
    # ========================================================

    def calculate_fk(self):

        pose = pose_from_fk(
            self.robot.q
        )

        self.status.text = (
            f"FK -> X={pose[0]:.2f}, "
            f"Y={pose[1]:.2f}, "
            f"Z={pose[2]:.2f}"
        )


    # ========================================================
    # IK
    # ========================================================

    def solve_ik(self):

        try:

            values = [
                float(field.text_value)
                for field in self.ik_fields
            ]

            x, y, z = values[:3]

        except ValueError:

            self.status.text = (
                "Invalid IK input"
            )

            return

        solution = inverse_kinematics(
            x,
            y,
            z
        )

        if solution is None:

            self.status.text = (
                "Target unreachable or "
                "outside joint limits"
            )

            return

        self.robot.q = np.array(
            solution
        )

        # Update sliders
        for i in range(3):

            self.sliders[i].double_value = (
                self.robot.q[i]
            )

        self.update_robot()

        self.status.text = (
            f"IK solved: "
            f"[{solution[0]:.1f}, "
            f"{solution[1]:.1f}, "
            f"{solution[2]:.1f}] degrees"
        )


    # ========================================================
    # AGENT TASK EXECUTION
    # ========================================================

    def start_animation(self):

        """
        Example sequence of Cartesian targets.

        The agent converts every target into joint angles
        using IK and then smoothly interpolates between
        joint configurations.
        """

        targets = [
            [2.5, 0.0, 2.0],
            [2.0, 1.5, 2.2],
            [1.0, 2.0, 1.8],
            [0.0, 2.5, 2.0],
            [-1.5, 1.5, 2.2],
            [-2.0, 0.0, 2.0],
            [0.0, 0.0, 2.5]
        ]

        self.animation_targets = []

        for target in targets:

            q = inverse_kinematics(
                target[0],
                target[1],
                target[2]
            )

            if q is not None:

                self.animation_targets.append(
                    np.array(q)
                )

        if len(self.animation_targets) == 0:

            self.status.text = (
                "No reachable targets"
            )

            return

        self.animation_index = 0

        self.animation_from = (
            self.robot.q.copy()
        )

        self.animation_to = (
            self.animation_targets[0]
        )

        self.animation_start = time.time()

        self.animation_running = True

        self.status.text = (
            "Agent executing target sequence..."
        )

        self.app.post_to_main_thread(
            self.window,
            self.animation_step
        )


    def animation_step(self):

        if not self.animation_running:
            return

        elapsed = time.time() - self.animation_start

        duration = 1.5

        progress = elapsed / duration

        progress = min(
            progress,
            1.0
        )

        # Smoothstep interpolation
        smooth = (
            progress * progress *
            (3 - 2 * progress)
        )

        self.robot.q = (
            self.animation_from
            + (
                self.animation_to
                - self.animation_from
            ) * smooth
        )

        for i in range(3):

            self.sliders[i].double_value = (
                self.robot.q[i]
            )

        self.update_robot()

        if progress >= 1.0:

            self.animation_index += 1

            if (
                self.animation_index
                >= len(self.animation_targets)
            ):

                self.animation_running = False

                self.status.text = (
                    "Agent task completed"
                )

                return

            self.animation_from = (
                self.robot.q.copy()
            )

            self.animation_to = (
                self.animation_targets[
                    self.animation_index
                ]
            )

            self.animation_start = time.time()

        self.app.post_to_main_thread(
            self.window,
            lambda: self.schedule_animation()
        )


    def schedule_animation(self):

        if self.animation_running:

            self.animation_step()


    # ========================================================
    # LAYOUT
    # ========================================================

    def on_layout(self, layout_context):

        rect = self.window.content_rect

        panel_width = 330

        self.panel.frame = o3d.visualization.gui.Rect(
            rect.x,
            rect.y,
            panel_width,
            rect.height
        )

        self.scene.frame = o3d.visualization.gui.Rect(
            rect.x + panel_width,
            rect.y,
            rect.width - panel_width,
            rect.height
        )


# ============================================================
# CLASSIC VISUALIZER FALLBACK (Tkinter + Open3D Visualizer)
# ============================================================

def run_classic_visualizer():
    import tkinter as tk
    from tkinter import messagebox

    robot = RobotArm()

    vis = o3d.visualization.Visualizer()
    vis.create_window("3-DOF Robot Arm (Classic Visualizer)", 1024, 768)

    # Setup scene
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(frame)

    # Ground grid
    grid = o3d.geometry.LineSet()
    grid_points = []
    grid_lines = []
    size, step, idx = 6, 0.5, 0
    for i in np.arange(-size, size + step, step):
        grid_points.extend([[i, -size, 0], [i, size, 0], [-size, i, 0], [size, i, 0]])
        grid_lines.extend([[idx, idx + 1], [idx + 2, idx + 3]])
        idx += 4
    grid.points = o3d.utility.Vector3dVector(grid_points)
    grid.lines = o3d.utility.Vector2iVector(grid_lines)
    grid.colors = o3d.utility.Vector3dVector([[0.4, 0.4, 0.4] for _ in range(len(grid_lines))])
    vis.add_geometry(grid)


    target_mesh = create_sphere(position=[2.5, 0.0, 2.0], radius=0.15, color=[1.0, 0.2, 0.2])
    vis.add_geometry(target_mesh)

    curr_robot_meshes = robot.get_geometries()
    for mesh in curr_robot_meshes:
        vis.add_geometry(mesh, reset_bounding_box=False)

    opt = vis.get_render_option()
    opt.background_color = np.array([0.08, 0.09, 0.12])
    opt.line_width = 2.0

    vis.poll_events()
    vis.update_renderer()

    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 1.5])
    ctr.set_front([1.5, -1.5, 1.0])
    ctr.set_up([0, 0, 1])
    ctr.set_zoom(0.7)


    # Tkinter Control Window
    root = tk.Tk()
    root.title("Robot Control Panel")
    root.geometry("340x520")
    root.attributes('-topmost', True)

    tk.Label(root, text="3-DOF ROBOT ARM", font=("Helvetica", 14, "bold")).pack(pady=10)

    sliders = []
    updating_sliders = False

    def update_robot_viz():
        nonlocal curr_robot_meshes
        for mesh in curr_robot_meshes:
            vis.remove_geometry(mesh, reset_bounding_box=False)
        curr_robot_meshes = robot.get_geometries()
        for mesh in curr_robot_meshes:
            vis.add_geometry(mesh, reset_bounding_box=False)


    def on_slider_change(val):
        nonlocal updating_sliders
        if updating_sliders:
            return
        robot.q = np.array([s.get() for s in sliders])
        update_robot_viz()

    frame_joints = tk.LabelFrame(root, text="Joint Angles (Degrees)", padx=10, pady=10)
    frame_joints.pack(fill="x", padx=10, pady=5)

    for i in range(3):
        tk.Label(frame_joints, text=f"Theta {i+1}").pack(anchor="w")
        s = tk.Scale(frame_joints, from_=JOINT_MIN[i], to=JOINT_MAX[i], orient="horizontal", command=on_slider_change, resolution=0.1)
        s.set(robot.q[i])
        s.pack(fill="x")
        sliders.append(s)

    # IK Section
    frame_ik = tk.LabelFrame(root, text="Cartesian Target (IK)", padx=10, pady=10)
    frame_ik.pack(fill="x", padx=10, pady=5)

    entries = {}
    for idx, (lbl, default_val) in enumerate([("X", "2.5"), ("Y", "0.0"), ("Z", "2.0")]):
        row = tk.Frame(frame_ik)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=lbl, width=3).pack(side="left")
        e = tk.Entry(row)
        e.insert(0, default_val)
        e.pack(side="right", expand=True, fill="x")
        entries[lbl] = e

    def solve_ik():
        nonlocal updating_sliders
        try:
            x = float(entries["X"].get())
            y = float(entries["Y"].get())
            z = float(entries["Z"].get())
            q_sol = inverse_kinematics(x, y, z)
            if q_sol is None:
                messagebox.showerror("IK Error", "Target position unreachable or out of joint limits!")
                return
            robot.q = q_sol
            updating_sliders = True
            for i in range(3):
                sliders[i].set(robot.q[i])
            updating_sliders = False
            update_robot_viz()
            target_mesh.translate(np.array([x, y, z]) - target_mesh.get_center(), relative=False)
            vis.update_geometry(target_mesh)
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers for X, Y, Z.")

    btn_ik = tk.Button(frame_ik, text="Solve IK", command=solve_ik, bg="#4CAF50", fg="white", font=("Helvetica", 10, "bold"))
    btn_ik.pack(fill="x", pady=5)

    # Animation
    anim_running = False
    anim_targets = []
    anim_index = 0
    anim_start = 0
    anim_from = None
    anim_to = None

    def start_animation():
        nonlocal anim_running, anim_targets, anim_index, anim_start, anim_from, anim_to
        targets = [
            [2.5, 0.0, 2.0],
            [1.5, 1.5, 1.8],
            [0.0, 2.5, 1.2],
            [-1.5, 1.5, 2.2],
            [-2.5, 0.0, 2.0],
            [2.5, 0.0, 2.0]
        ]
        q_targets = []
        for t in targets:
            q_sol = inverse_kinematics(t[0], t[1], t[2])
            if q_sol is not None:
                q_targets.append(q_sol)
        if not q_targets:
            return
        anim_targets = q_targets
        anim_index = 0
        anim_from = robot.q.copy()
        anim_to = anim_targets[0]
        anim_start = time.time()
        anim_running = True

    btn_anim = tk.Button(root, text="Run Target Sequence", command=start_animation, bg="#2196F3", fg="white", font=("Helvetica", 10, "bold"))
    btn_anim.pack(fill="x", padx=10, pady=10)

    def main_loop():
        nonlocal anim_running, anim_index, anim_start, anim_from, anim_to, updating_sliders
        if anim_running:
            elapsed = time.time() - anim_start
            duration = 2.0
            progress = min(elapsed / duration, 1.0)
            smooth = progress * progress * (3 - 2 * progress)
            robot.q = anim_from + (anim_to - anim_from) * smooth
            updating_sliders = True
            for i in range(3):
                sliders[i].set(robot.q[i])
            updating_sliders = False
            update_robot_viz()
            if progress >= 1.0:
                anim_index += 1
                if anim_index >= len(anim_targets):
                    anim_running = False
                else:
                    anim_from = robot.q.copy()
                    anim_to = anim_targets[anim_index]
                    anim_start = time.time()

        vis.poll_events()
        vis.update_renderer()
        root.after(20, main_loop)

    root.after(20, main_loop)
    root.mainloop()
    vis.destroy_window()


# ============================================================
# MAIN
# ============================================================

def main():
    print("[INFO] Starting 3-DOF Robot Arm Simulation using Classic Visualizer...")
    run_classic_visualizer()


if __name__ == "__main__":
    main()
