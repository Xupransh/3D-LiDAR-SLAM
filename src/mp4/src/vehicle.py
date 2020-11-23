import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel
import time


if __name__ == "__main__":
    rospy.init_node("model_dynamics")
    model = bicycleModel()

    endList = 0


    pos_list = [[100,53],[80,57],[60,56],[50,57],[40,58],[35,55],[34,44],[40,39],[45,40],[55,40],[68,40],[75,30],[75,28],[83,22],[104,22],[110,34],[102,39],[96,47]]
    pos_idx = 0

    targetState = ModelState()
    targetState.pose.position.x = pos_list[pos_idx][0] + 15 - 100
    targetState.pose.position.y = pos_list[pos_idx][1] + 100 - 100

    start = time.time()
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        currState =  model.getModelState()
        if not currState.success:
            continue


        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)


        if(distToTargetX < 1 and distToTargetY < 1):
            pos_idx = pos_idx+1
            pos_idx = pos_idx % len(pos_list)
            targetState = ModelState()
            targetState.pose.position.x = pos_list[pos_idx][0] + 15 - 100
            targetState.pose.position.y = pos_list[pos_idx][1] + 100 - 100
            print("reached",pos_list[pos_idx][0],pos_list[pos_idx][1])
        else:
            model.setModelState(currState, targetState)

    rospy.spin()
