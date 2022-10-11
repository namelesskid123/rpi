from RPi.src.communicator.MultiProcessCommunication import MultiProcessCommunicator
from src.task2.commands import LEFT_1ST, LEFT_2ND, RIGHT_1ST, RIGHT_2ND, FORWARD, REVERSE

class Task2:
    def __init__(self):
        self.DEFAULT_DIST_AWAY = 35
        self.cur_dist_away:float = -1
        self.cur_img_result:str = ""
        self.multiprocess_communication_process = MultiProcessCommunicator()
        self.multiprocess_communication_process.start()

    def set_cur_dist_away(self, dist:float):
        self.cur_dist_away = dist

    def set_img_result(self, result:str):
        self.cur_img_result = result

    def move_next(self, ):
        pass

    def move_to_default_dist_away(self):
        if (self.cur_dist_away > self.DEFAULT_DIST_AWAY):
            dist_to_move = self.cur_dist_away - self.DEFAULT_DIST_AWAY
            # send move forward command
        else:
            dist_to_move = self.DEFAULT_DIST_AWAY - self.cur_dist_away
            # send move backward command
        # wait for the robot to move
        command_res = None
        while (command_res is None):
            pass

    def run(self):
        print("Visiting the 1st obstalce")
        
        while (self.cur_dist_away == -1):
            pass

        self.move_to_default_dist_away()
        # start image recognition for the 1st image
        while (self.cur_img_result == ""):
            pass
        
        # send command to move to the 2nd obstacle
        print("Visiting the 2nd obstalce")
        
        self.move_to_default_dist_away()
        print("Checking if correction is needed for the 2nd obstalce")
        print("Going back to the car park")
        
    