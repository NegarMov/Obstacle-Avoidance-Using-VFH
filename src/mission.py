#!/usr/bin/python3

import rospy
from fp_step1.srv import GetNextDestination, GetNextDestinationResponse

class Mission():

    def __init__(self) -> None:

        self.service = rospy.Service(
            'GetNextDestination', GetNextDestination, self.get_next_destination
        )

        self.checkpoints_reached = 0
        self.goals = [(2.5, 2), (3, 6.25), (5.75, 5), (13, 6.75)]


    def get_next_destination(self, req):

        rospy.loginfo(f"Mission service called")

        res = GetNextDestinationResponse()

        if self.checkpoints_reached < len(self.goals):
            res.next_x = self.goals[self.checkpoints_reached][0]
            res.next_y = self.goals[self.checkpoints_reached][1]
            res.last_goal = False

            self.checkpoints_reached += 1
        
        else:
            res.last_goal = True

        return res

if __name__ == "__main__":
    rospy.init_node("mission", anonymous=True)

    mission = Mission()

    rospy.spin()