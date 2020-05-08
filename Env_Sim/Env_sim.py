import gym
import envs
import time
import cv2

env = gym.make('CustomEnv-v0')
n=0

while True:
    #env.step()
    env.render()

    k=cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    elif k == ord('w'):
        if env.agent[n].agent_pos[0]>0 and env.agent[n].map_check(env.agent[n].agent_pos[0]-1,env.agent[n].agent_pos[1])!=0 :
            env.agent[n].odom_update( [env.agent[n].agent_pos[0]-1,env.agent[n].agent_pos[1]] )
    elif k == ord('s'):
        if env.agent[n].agent_pos[0]+1<env.size[0] and env.agent[n].map_check(env.agent[n].agent_pos[0]+1,env.agent[n].agent_pos[1])!=0 :
            env.agent[n].odom_update( [env.agent[n].agent_pos[0]+1,env.agent[n].agent_pos[1]] )
    elif k == ord('a'):
        if env.agent[n].agent_pos[1]>0 and env.agent[n].map_check(env.agent[n].agent_pos[0],env.agent[n].agent_pos[1]-1)!=0 :
            env.agent[n].odom_update( [env.agent[n].agent_pos[0],env.agent[n].agent_pos[1]-1] )
    elif k == ord('d'):
        if env.agent[n].agent_pos[1]+1<env.size[1] and env.agent[n].map_check(env.agent[n].agent_pos[0],env.agent[n].agent_pos[1]+1)!=0 :
            env.agent[n].odom_update( [env.agent[n].agent_pos[0],env.agent[n].agent_pos[1]+1] )
    elif k == ord('f'):
        env.agent[n].step()
    elif k == ord('m'):
        for x in env.largest_contour:
			env.agent[n].odom_update([x[0],x[1]])
			env.render()
			if cv2.waitKey(1) & 0xFF == ord('x'):
				break
    elif k >= ord('0') and k <= ord('9'):
        _n=k-ord('0')
        if _n< env.agents_count:
            n=_n

env.close()
