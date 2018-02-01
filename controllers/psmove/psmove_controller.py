import numpy as np


class PSmoveController():
    previous_pos = (0.0, 0.0, 0.0)
    previous_orientation = (0.0, 0.0, 0.0)
    init_pos = False
    init_orientation = False
    CONTROLLER_SCALE = 0.05
    X_BUTTON_CODE = 64
    X_SCALE = 640
    Y_SCALE = 480
    R_SCALE = 200

    def __init__(self):
        pass

    def get_diff_position(self, msg):
        if not self.init_pos:
            self.previous_pos = msg.data[0:3]
            self.init_pos = True
        curr_pos_command = msg.data[0:3]

        diff_pos = np.multiply(np.subtract(curr_pos_command, self.previous_pos), self.CONTROLLER_SCALE)
        temp = diff_pos[2]
        diff_pos[2] = -diff_pos[1]
        diff_pos[1] = temp
        self.previous_pos = msg.data[0:3]

        return np.sign(diff_pos)

    def get_position(self, msg):
        curr_pos_command = msg.data[0:3]

        pos = np.multiply(curr_pos_command, self.CONTROLLER_SCALE)
        temp = pos[2]
        pos[2] = pos[1]
        pos[1] = temp

        pos[0] = (pos[0] / self.X_SCALE) * 2 - 1
        pos[1] = ((pos[1] - 8) / self.R_SCALE) * 2 - 1 + 0.31
        pos[2] = (pos[2] / self.Y_SCALE) * 2 - 1 - 0.49
        return pos

    def get_orientation(self, msg):
        if not self.init_orientation:
            self.previous_orientation = msg.data[3:6]
            self.init_orientation = True
        curr_orientation_command = msg.data[3:6]

        diff_orientation = np.multiply(np.subtract(curr_orientation_command,
                                                   self.previous_orientation), 40.0)
        self.previous_orientation = msg.data[3:6]
        return [0.0, 0.0, 0.0]

    def get_button(self, msg):
        if msg.data[6] == self.X_BUTTON_CODE:
            return "gripper-toggle"
        else:
            return "undefined-action"
