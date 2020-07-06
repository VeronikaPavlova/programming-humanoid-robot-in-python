'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion
* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder
* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

from pid import PIDAgent
import time
from keyframes import leftBackToStand, hello, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.startTime = -1
        self.done = False

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        if self.done:
            return {}

        names, times, keys = keyframes

        if self.startTime == -1:
            self.startTime = perception.time

        current_time = perception.time - self.startTime

        if current_time > 10.5:
            if perception.imu[1] > -0.5:
                print("NAO is standing")
                self.done = True
            else:
                print("Nao is not standing. Try again!")
            return {}

        angle = 0
        # iterate through joins
        for (j, name) in enumerate(names):
            # 24 joints but not all used, also need to adjust later the RHipYawPitch (should be same as LHipYawPicth
            if name in self.joint_names:
                for (t_inx, time) in enumerate(times[j]):
                    # First interpolation angle
                    if current_time < times[j][0]:
                        angle = self.first_bezier_angle(j, name, current_time, times, keys)
                    # Now is between interpolation
                    elif t_inx + 1 < len(times[j]):
                        if times[j][t_inx] < current_time < times[j][t_inx + 1]:
                            angle = self.calculate_bezier_angle(j, t_inx, current_time, times, keys)
                    target_joints[name] = angle
                    # LHip and RHip are connected, but not in the simulation
                    if name == "LHipYawPitch":
                        target_joints["RHipYawPitch"] = angle

        return target_joints

    def first_bezier_angle(self, j, name, current_time, times, keys):

        t0 = 0.0
        t3 = times[j][0]  # first time when interpolation start

        dt = (current_time - t0) / (t3 - t0)

        # calculate points - first point is current joint state
        p0 = self.perception.joint[name]
        p3 = keys[j][0][0]

        # handle points
        p1 = keys[j][0][1][2] + p0
        p2 = keys[j][0][2][2] + p3

        return self.cubic_bezier_interpolation(p0, p1, p2, p3, dt)

    def calculate_bezier_angle(self, j, t, current_time, times, keys):

        # Time values
        t_0 = times[j][t]
        t_3 = times[j][t + 1]

        dt = (current_time - t_0) / (t_3 - t_0)

        # Angles
        p0 = keys[j][t][0]
        p3 = keys[j][t + 1][0]
        # Control angles
        p1 = p0 + keys[j][t][1][2]
        p2 = p3 + keys[j][t][1][2]

        return self.cubic_bezier_interpolation(p0, p1, p2, p3, dt)

    @staticmethod
    def cubic_bezier_interpolation(p0, p1, p2, p3, i):
        t0 = (1 - i) ** 3
        t1 = 3 * i * (1 - i) ** 2
        t2 = 3 * (1 - i) * i ** 2
        t3 = i ** 3
        return t0 * p0 + t1 * p1 + t2 * p2 + t3 * p3


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES

    # By the Belly keyframes NAO not falls on the Belly ...
    if agent.keyframes == leftBellyToStand() or agent.keyframes == rightBellyToStand():
        agent.keyframes[2][0] = agent.keyframes[2][1]

    agent.run()
