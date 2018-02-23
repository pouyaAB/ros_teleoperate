config = {}

config['record_path'] = '/home/d3gan/development/datasets/record/real_time'
config['robot_command_file'] = '/home/d3gan/development/datasets/record/real_time/commands.csv'
config['image_size'] = 64
config['task'] = '5002'

config['camera_topics'] = ['/camera1/usb_cam1/image_raw', '/camera2/usb_cam2/image_raw', '/camera3/usb_cam3/image_raw']
config['cameras_switch'] = [False, True, False]
config['record_human'] = True

