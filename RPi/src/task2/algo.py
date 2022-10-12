from RPi.src.task2.constants import LEFT, PARK_DIST, STOP
from src.ultrasonic import distance
from RPi.src.communicator.MultiProcessCommunication import MultiProcessCommunicator
from src.task2.constants import LEFT_1ST, LEFT_2ND, RIGHT_1ST, RIGHT_2ND, FORWARD, REVERSE

class Task2:
    def __init__(self):
        self.DEFAULT_DIST_AWAY = 35
        self.cur_dist_away:float = -1
        self.cur_img_result:str = ""
        self.multiprocess_communication_process = MultiProcessCommunicator()
        self.multiprocess_communication_process.start()

    def update_cur_dist_away(self):
        self.cur_dist_away = distance() #ultrasonic dist

    def clear_cur_dist_away(self):
        self.cur_dist_away = -1

    def update_img_result(self):
        res = self.multiprocess_communication_process.get_img_result()
        self.cur_img_result = res if res is not None else ""

    def clear_img_result(self):
        self.cur_img_result = ""

    def move_next(self, obstacle_id):
        if obstacle_id == 1:
            if self.cur_img_result == LEFT:
                self.multiprocess_communication_process.add_STM_command_to_queue(LEFT_1ST)
            else:
                self.multiprocess_communication_process.add_STM_command_to_queue(RIGHT_1ST)
        elif obstacle_id == 2:
            if self.cur_img_result == LEFT:
                self.multiprocess_communication_process.add_STM_command_to_queue(LEFT_2ND)
            else:
                self.multiprocess_communication_process.add_STM_command_to_queue(RIGHT_2ND)

    def move_to_default_dist_away_static(self):
        if (self.cur_dist_away > self.DEFAULT_DIST_AWAY):
            dist_to_move = self.cur_dist_away - self.DEFAULT_DIST_AWAY
            # send move forward command
            forward_cmd = FORWARD + int(dist_to_move)
            print("Sending forward command: " + forward_cmd)
            self.multiprocess_communication_process.add_STM_command_to_queue(forward_cmd)
        else:
            dist_to_move = self.DEFAULT_DIST_AWAY - self.cur_dist_away
            reverse_cmd = REVERSE + int(dist_to_move)
            print("Sending reverse command: " + reverse_cmd)
            self.multiprocess_communication_process.add_STM_command_to_queue(reverse_cmd)
        # wait for the robot to complete movements
        while not self.multiprocess_communication_process.robot_command_completed:
            pass
        self.multiprocess_communication_process.clear_command_complete_status()

    def move_to_default_dist_away_dynamic(self):
        dist = distance()

        if dist > self.DEFAULT_DIST_AWAY:
            while dist > self.DEFAULT_DIST_AWAY:
                self.multiprocess_communication_process.add_STM_command_to_queue(FORWARD)
                dist = distance()
            self.multiprocess_communication_process.add_STM_command_to_queue(STOP)
            # wait for the robot to complete movements
            while not self.multiprocess_communication_process.robot_command_completed:
                pass
            self.multiprocess_communication_process.clear_command_complete_status()
        elif dist < self.DEFAULT_DIST_AWAY:
            while dist < self.DEFAULT_DIST_AWAY:
                self.multiprocess_communication_process.add_STM_command_to_queue(REVERSE)
                dist = distance()
            self.multiprocess_communication_process.add_STM_command_to_queue(STOP)
            # wait for the robot to complete movements
            while not self.multiprocess_communication_process.robot_command_completed:
                pass
            self.multiprocess_communication_process.clear_command_complete_status()

    def park(self):
        dist = distance()
        while dist > PARK_DIST:
            self.multiprocess_communication_process.add_STM_command_to_queue(FORWARD)
            dist = distance()
        self.multiprocess_communication_process.add_STM_command_to_queue(STOP)
        # wait for the robot to complete movements
        while not self.multiprocess_communication_process.robot_command_completed:
            pass
        self.multiprocess_communication_process.clear_command_complete_status()
        print("Parking complete")

    def run(self):
        print("Pending start command from Android")
        while self.multiprocess_communication_process.start_task_2 is False:
            pass
        print("Start command received!")
        print("Visiting the 1st obstalce")
        # get initial distance away from the obstacle
        while self.cur_dist_away == -1:
            self.update_cur_dist_away()
        # move to the default distance away from the obstacle
        self.move_to_default_dist_away()
        self.clear_cur_dist_away()
        # start image recognition for the 1st image

        # TODO: implement an image recognition function in MultiProcessCommunication

        while self.cur_img_result == "":
            self.update_img_result()
        # send command to move to the 2nd obstacle
        print("Visiting the 2nd obstalce")

        self.move_next(self.cur_img_result, 1)
        # clean up the image result
        self.clear_img_result()

        self.move_to_default_dist_away_dynamic()
        # start image recognition for the 2nd image

        # TODO: implement an image recognition function in MultiProcessCommunication

        while self.cur_img_result == "":
            self.update_img_result()

        self.move_next(self.cur_img_result, 2)

        print("Robot should automatically go back to the car park")
        while not self.multiprocess_communication_process.robot_ready_to_park:
            pass
        print("Robot is ready to park")
        self.park()