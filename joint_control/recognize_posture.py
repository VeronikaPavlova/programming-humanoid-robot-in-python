'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''

from angle_interpolation import AngleInterpolationAgent
from keyframes import leftBackToStand, hello, leftBellyToStand, rightBackToStand
from os import listdir, path

import pickle
import numpy as np


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = None

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'

        file = open('robot_pose.pkl', 'rb')
        self.posture_classifier = pickle.load(file)
        file.close()

        posture = self.posture_classifier.predict(np.asarray(self.get_actual_posture_data(perception)))
        posture_name = listdir('robot_pose_data')[posture[0]]

        return posture_name

    @staticmethod
    def get_actual_posture_data(perception):
        posture_data = np.array([perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'],
                        perception.joint['LKneePitch'], perception.joint['RHipYawPitch'], perception.joint['RHipRoll'],
                        perception.joint['RHipPitch'], perception.joint['RKneePitch'], perception.imu[0],
                        perception.imu[1]])

        return posture_data.reshape(1, -1)

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
