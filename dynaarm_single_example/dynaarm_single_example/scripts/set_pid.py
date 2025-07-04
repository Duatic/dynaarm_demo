from textual.app import App, ComposeResult
from textual.containers import Vertical, Horizontal
from textual.widgets import Input, Label
import subprocess


class PIDTunerPanel(Vertical):
    """A panel for tuning PID values for a single joint."""

    def __init__(self, joint_name):
        super().__init__()
        self.joint_name = joint_name
        self.node_name = "/pid_tuner"
        self.p_gain_input = None
        self.i_gain_input = None
        self.d_gain_input = None

    def get_current_param(self, param):
        """Retrieve the current parameter value from ROS 2."""
        try:
            result = subprocess.run(
                ["ros2", "param", "get", self.node_name, param], capture_output=True, text=True
            )
            value = result.stdout.strip().split(":")[-1].strip()
            return value if value else "0.0"
        except Exception:
            return "0.0"

    def set_param(self, param, value):
        """Set a new parameter value in ROS 2."""
        try:
            float_value = float(value)  # Ensure it's a float
            subprocess.run(["ros2", "param", "set", self.node_name, param, str(float_value)])
        except ValueError:
            print(f"Invalid float value: {value}")

    def compose(self) -> ComposeResult:
        yield Label(f"[blue]{self.joint_name}[/blue]", classes="title")

        self.p_gain_input = Input(
            value=self.get_current_param(f"{self.joint_name}/p_gain"), name="P Gain"
        )
        self.i_gain_input = Input(
            value=self.get_current_param(f"{self.joint_name}/i_gain"), name="I Gain"
        )
        self.d_gain_input = Input(
            value=self.get_current_param(f"{self.joint_name}/d_gain"), name="D Gain"
        )

        yield Vertical(
            Label("P Gain"),
            self.p_gain_input,
            Label("I Gain"),
            self.i_gain_input,
            Label("D Gain"),
            self.d_gain_input,
        )

    def on_input_submitted(self, event: Input.Submitted) -> None:
        """Handle value submission (Enter key)."""
        param_map = {
            self.p_gain_input: f"{self.joint_name}/p_gain",
            self.i_gain_input: f"{self.joint_name}/i_gain",
            self.d_gain_input: f"{self.joint_name}/d_gain",
        }

        if event.input in param_map:
            self.set_param(param_map[event.input], event.value)
            event.input.border_title = f"✔ Updated {event.value}"
            event.input.refresh()


class PIDTunerApp(App):
    """Main application containing panels for multiple joints."""

    def compose(self) -> ComposeResult:
        yield Label("A panel for tuning PID values for a single joint.", classes="header")
        yield Label("Enter new values and press [green]Enter[/green] to update.")
        yield Label(" ", classes="header")
        yield Vertical(
            # Shoulder Actuators
            Horizontal(
                PIDTunerPanel("shoulder_rotation"),
                PIDTunerPanel("shoulder_flexion"),
                PIDTunerPanel("elbow_flexion"),
            ),
            # Wrist Actuators
            Horizontal(
                PIDTunerPanel("forearm_rotation"),
                PIDTunerPanel("wrist_flexion"),
                PIDTunerPanel("wrist_rotation"),
            ),
        )
        yield Label("Press [red]ESC[/red] to leave.", classes="footer")

    def on_key(self, event) -> None:
        """Close the app when Escape is pressed."""
        if event.key == "escape":
            self.exit()


if __name__ == "__main__":
    app = PIDTunerApp()
    app.run()
