from src.ultrasonic import distance
from src.task2.constants import LEFT_1ST, LEFT_2ND, RIGHT_1ST, RIGHT_2ND, FORWARD, REVERSE, LEFT, PARK_DIST, STOP, RIGHT

class Task2:
    def __init__(self, _add_STM_command_to_queue, _take_pic, _update_img_deque):
        self.DEFAULT_DIST_AWAY = 35
        self.cur_dist_away:float = -1
        self.cur_img_result:str = ""

        self.robot_command_completed = False
        self.robot_ready_to_park = False

        self.add_STM_command_to_queue = _add_STM_command_to_queue
        self.take_pic = _take_pic
        self.update_img_deque = _update_img_deque

    def set_is_robot_ready_to_park(self, status):
        self.robot_ready_to_park = status

    def is_robot_ready_to_park(self):
        return self.robot_ready_to_park

    def is_command_compeleted(self):
        return self.robot_command_completed == True

    def set_command_complete_status(self, status):
        self.robot_command_completed = status

    def clear_command_complete_status(self):
        self.robot_command_completed = False

    def update_cur_dist_away(self):
        self.cur_dist_away = distance() #ultrasonic dist

    def clear_cur_dist_away(self):
        self.cur_dist_away = -1

    def set_img_result(self, img_result):
        self.cur_img_result = img_result if img_result is not None else ""

    def get_img_result(self):
        return self.cur_img_result
    
    def clear_img_result(self):
        self.cur_img_result = ""

    def move_next(self, obstacle_id):
        if obstacle_id == 1:
            if self.cur_img_result == LEFT:
                self.add_STM_command_to_queue(LEFT_1ST)
            else:
                self.add_STM_command_to_queue(RIGHT_1ST)
        elif obstacle_id == 2:
            if self.cur_img_result == LEFT:
                self.add_STM_command_to_queue(LEFT_2ND)
            else:
                self.add_STM_command_to_queue(RIGHT_2ND)

    # pass callback from mpc
    def move_to_default_dist_away_static(self):
        if (self.cur_dist_away > self.DEFAULT_DIST_AWAY):
            dist_to_move = self.cur_dist_away - self.DEFAULT_DIST_AWAY
            # send move forward command
            forward_cmd = FORWARD + int(dist_to_move)
            print("Sending forward command: " + forward_cmd)
            self.add_STM_command_to_queue(forward_cmd.encode())
        else:
            dist_to_move = self.DEFAULT_DIST_AWAY - self.cur_dist_away
            reverse_cmd = REVERSE + int(dist_to_move)
            print("Sending reverse command: " + reverse_cmd)
            self.add_STM_command_to_queue(reverse_cmd.encode())
        # wait for the robot to complete movements
        while not self.is_command_compeleted():
            pass
        self.clear_command_complete_status()

    def move_to_default_dist_away_dynamic(self):
        dist = distance()

        if dist > self.DEFAULT_DIST_AWAY:
            while dist > self.DEFAULT_DIST_AWAY:
                self.add_STM_command_to_queue(FORWARD.encode())
                dist = distance()
            self.add_STM_command_to_queue(STOP.encode())
            # wait for the robot to complete movements
            while not self.is_command_compeleted():
                pass
            self.clear_command_complete_status()
        elif dist < self.DEFAULT_DIST_AWAY:
            while dist < self.DEFAULT_DIST_AWAY:
                self.add_STM_command_to_queue(REVERSE.encode())
                dist = distance()
            self.add_STM_command_to_queue(STOP.encode())
            # wait for the robot to complete movements
            while not self.is_command_compeleted():
                pass
            self.clear_command_complete_status()

    def park(self):
        dist = distance()
        while dist > PARK_DIST:
            self.add_STM_command_to_queue(FORWARD.encode())
            dist = distance()
        self.add_STM_command_to_queue(STOP.encode())
        # wait for the robot to complete movements
        while not self.is_command_compeleted():
            pass
        self.clear_command_complete_status()
        print("Parking complete")

    def run(self):
        print("Start command received!")
        print("Visiting the 1st obstacle")
        # get initial distance away from the obstacle
        while self.cur_dist_away == -1:
            self.update_cur_dist_away()
        # move to the default distance away from the obstacle
        self.move_to_default_dist_away()
        self.clear_cur_dist_away()
        # start image recognition for the 1st image

        image = self.take_pic()
        print('Picture taken')
        message = 'C[0]'
        self.update_img_deque(image, message)
        
        while self.get_img_result() == "":
            self.update_img_result()
        # send command to move to the 2nd obstacle
        print("Visiting the 2nd obstalce")

        self.move_next(self.cur_img_result, 1)
        # clean up the image result
        self.clear_img_result()

        self.move_to_default_dist_away_dynamic()
        # start image recognition for the 2nd image

        image = self.take_pic()
        print('Picture taken')
        message = 'C[1]'
        self.update_img_deque(image, message)

        while self.get_img_result() == "":
            self.update_img_result()

        self.move_next(self.cur_img_result, 2)

        print("Robot should automatically go back to the car park")
        while not self.is_robot_ready_to_park():
            pass
        print("Robot is ready to park")
        self.park()
        print("Robot is parked")
        
