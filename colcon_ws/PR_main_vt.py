"""
This code is the PC Client
normal to compliant to recovery 
recorvery to compliant


"""
#import com_test # mode 0high -- low1-pressure + low1-sensor + low2-pressure
import PR_Computing_task
#import vtech
#import static_test
# import sensor_test # mode 1
# import ramp_test # mode 2
# import step_test # mode 3
# from time import time,sleep
import numpy as np
from time import sleep
def main():
    try:
        #### Select control method ####
        # number of elements
        #print("\nList is - ", a)

        flag_ctrl_mode=1

        if flag_ctrl_mode==1:
            #p_client=com_test.pc_client()
        #elif f #lag_ctrl_mode==1:
            #p_client.NArs = a
            p_client=PR_Computing_task.pc_client()
            #p_client=vtech.pc_client()
        #elif flag_ctrl_mode==2:
          #  p_client=static_test.pc_client()
        # elif flag_ctrl_mode==3:
        #     p_client=step_test.pc_client()
        p_client.positionProfile_flag=3 
        p_client.flag_use_mocap=1
        p_client.trailDuriation=120.


        # p_client.rampRateAbs=np.radians(0.5) # 1 deg/sec
        # p_client.rampAmpAbs=np.radians(15) # x1(t0)-rampAmp
        # p_client.rampFlatTime=5.0 # sec



        p_client.th2.start()
        sleep(0.5)
        p_client.th1.start()
        sleep(0.5)
        p_client.th3.start()
        sleep(0.5)
        #p_client.th4.start()
        while 1:
            pass
    except KeyboardInterrupt:
        p_client.th1_flag=False
        p_client.th2_flag=False
        exit()
        
if __name__ == '__main__':
    main()
