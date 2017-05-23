import numpy as np


class PS4Controller():
    CONTROLLER_SCALE = 0.05

    NO_BUTTON_PUSHED = True
    def __init__(self):
        pass

    def get_diff_position(self, msg):
        return self.get_position(msg)

    def get_position(self, msg):
        axes = msg.axes
        up_down = 0.0
        if axes[3] == -1.0:
            up_down = 1.0
        elif axes[4] == -1.0:
            up_down = -1.0
        pos = [axes[9], -axes[10], up_down]
        return pos


    def get_orientation(self, msg):
        axes = msg.axes
        buttons = msg.buttons
        up_down = 0.0
        if buttons[4] == 1.0:
            up_down = 1.0
        elif buttons[5] == 1.0:
            up_down = -1.0
        orientation = [-axes[5], axes[2], up_down]
        return orientation
        pass

    def get_button(self, msg):
        if self.NO_BUTTON_PUSHED and msg.buttons[1] == 1:
            self.NO_BUTTON_PUSHED = False
            return "gripper-toggle"
        elif self.NO_BUTTON_PUSHED and msg.buttons[3] == 1:
            self.NO_BUTTON_PUSHED = False
            return "triangle"
        elif self.NO_BUTTON_PUSHED and msg.buttons[2] == 1:
            self.NO_BUTTON_PUSHED = False
            return "circle"
        elif self.NO_BUTTON_PUSHED and msg.buttons[0] == 1:
            self.NO_BUTTON_PUSHED = False
            return "square"
        else:
            if sum(msg.buttons) == 0:
                self.NO_BUTTON_PUSHED = True
            return "undefined-action"
