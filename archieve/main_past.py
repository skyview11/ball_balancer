from tqdm import tqdm
import time
from core.motorDegreeCalculator import MotorDegreeCalculator as MotorDegreeCalculator
import random
from utils.functions import *
import cv2
from cvDetection.ballDeteectUtil import *
from DynanixelController import DynamixelController
calculator = MotorDegreeCalculator()


### OTHER CONSTANTS

Kp = 0.0005
k_v = 0.005
Kd = 0.0015

def getBallCenterCoordFromVideo(frame):
    """get ball center coordinate from vidio frame input

    Args:
        frame (??): video frame from the videoCapture

    Returns:
        ball_center (tuple?): 2 dim coordinate of the center of the ball coordinate. If no ball in the frame, returns (-1, -1)
    """
    
    ball_center = (-1, -1)
    
    img = np.flip(np.flip(np.array(frame), axis=0), axis=1)
            
    img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    threshold = 60
    filter_img = cv2.inRange(img, (0, 0, 0), (threshold, threshold, threshold))
    
    # 경계선 가져오기
    contours_simple = get_contours(filter_img, 50, True)
    
    ######################
    # 텍스트 출력하고 경계선 그리기(simple)
    simple_text = "contours count : " + str(len(contours_simple))
    simple_img = cv2.putText(img.copy(), simple_text, (0, 25), FONT, 1, RED)
    
    
    for cnt in contours_simple:
        
        cv2.drawContours(simple_img, cnt, -1, BLUE, 5)
        
        if is_circle(cnt):
            # 원형일 경우 빨간색으로 그리기
            ball_center = draw_points(simple_img, cnt, 0.01, RED)                      
            cv2.circle(simple_img, (int(ball_center[0]), int(ball_center[1])), 3, (255, 255, 255), -1)
            
    
    
    return ball_center, simple_img


def setTargetCoordinate():
    return (160, 130)
    is_target_done = 0
    
    while not is_target_done:
        
        _target_coord =list(map(int, input().split()))
        if (len(_target_coord) == 1):
            is_target_done = 1
        else:
            target_coord = _target_coord
    
    return target_coord


class VelocityManager:
    def __init__(self, n):
        self.past_errors: np.ndarray = np.zeros((n, 3)) # (x, y, time)
        self.cur_buff_idx = 0
    def addPastError(self, past_err:tuple, time:float):
        self.past_errors[self.cur_buff_idx] = past_err + (time,)
        self.cur_buff_idx += 1
    def addEmptyStack(self):
        self.past_errors[self.cur_buff_idx] = (0, 0, 0)
        self.cur_buff_idx += 1
        
        
    def getVelocity(self, cur_err_vec: tuple):
        time_interval = time.time() - self.past_errors[self.cur_buff_idx,2]
        vel_vec = [(cur_err_vec[0]-np.mean(self.past_errors[:,0]))/time_interval, 
                   (cur_err_vec[1]-np.mean(self.past_errors[:,1]))/time_interval]
        return vel_vec

def isBallExist(ball_center:tuple):
    return ball_center[0]!=-1 and ball_center[1] != -1
            
    
# main 함수 부분

if __name__ == '__main__':
    ########## INITISLIZATION ##########
    vector = [0,0,1]
    detect = 0
    error = 0
    
    roll=-1
    pitch=-1
    
    
    activate_flag = True
    start = time.time()
    counter = 0
    t_list = []
    prev_err_list = np.zeros((20, 3))
    recent_idx = 0
    no_ball_count = 0
    velocity_manager = VelocityManager(20)
    
    #### FLAG DEF ####
    CIRCLE_EXIST = 0
    
    ## WebCam Initialize
    webcam = cv2.VideoCapture(cv2.CAP_DSHOW+1)
    if not webcam.isOpened():
        print("Could not open webcam")
        exit()
    
    target_coord = setTargetCoordinate()
    
    # Dynamixel Initialize
    #dynamixelInit()
    import pdb;pdb.set_trace()
    dynamixel_controller = DynamixelController([512, 512, 512])
    while webcam.isOpened():
        # read webcam and get coordinate of ball center
        status, frame = webcam.read() 
        if not status:
            continue
        
        ball_center, simple_img = getBallCenterCoordFromVideo(frame)   
        cv2.circle(simple_img, (int(target_coord[0]), int(target_coord[1])), 3, GREEN, -1) ## add dot to target coordinate         
        cv2.imshow("test", simple_img)
        
        
        ### Update target normal vector if ball is detected, else set to (0, 0, 1)
        if (isBallExist(ball_center)):
            ## calculate error vector
            error_vector = [target_coord[0]-ball_center[0], target_coord[1]-ball_center[1]]  
            error_vector[0] = error_vector[0]//5*5
            error_vector[1] = error_vector[1]//5*5
            
            velocity_vector = velocity_manager.getVelocity(error_vector)
            target_normal_vector = ((error_vector[0])*Kp + (velocity_vector[0])*Kd, (error_vector[1])*Kp + (velocity_vector[1])*Kd, 1)
        else:
            target_normal_vector = (0, 0, 1)
        print("({:.2f}, {:.2f}, 1.00)".format(target_normal_vector[0], target_normal_vector[1]))
        #########################  PID  ###############################
        # n = getNorm(err_vec)
        
        
        # counter = (counter+1)%1000
        
        
        ## calculate velocity
        
        
        
        
        # if (np.any(prev_err_list[:, 2]==0)): # set velocity to 0 if ball is just detected
        #     print("a")
        #     vel_vec = [0, 0, 0]
        #     time_interval = 100
        # else:
        #     err_vec[0] = err_vec[0]//5*5;err_vec[1] = err_vec[1]//5*5
        #     time_interval = time.time() - prev_err_list[i,2]
        #     vel_vec = [(err_vec[0]-np.mean(prev_err_list[:,0]))/time_interval, (err_vec[0]-np.mean(prev_err_list[:,0]))/time_interval]
        #     print(getNorm(vel_vec), (prev_err_list))
        #     import pdb;pdb.set_trace()
        
        
        ## get final normal vector
        #vector = [(err_vec[0])*Kp + (vel_vec[0])*Kd, (err_vec[1])*Kp + (vel_vec[1])*Kd, 1]
        #vector = [(err_vec[0])*Kp, (err_vec[1])*Kp, 1]
        #vector = [(vel_vec[0])*Kd,(vel_vec[1])*Kd, 1]
        #vector = [0,0,1]
        
        # prev_err_list[recent_idx] = tuple(err_vec) + (time.time(),)
        # recent_idx = (recent_idx+1)%20
        
        calculator.setTargetNormalVector(target_normal_vector)
        angles = calculator.calculateMotorAngles()
        
        dy_pos = [int((angle * 180/math.pi + 150) * 1024/300) for angle in angles]

        ## Move Dynamixel to target position
        dynamixel_controller.moveGroupDynamixel(dy_pos)  

        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        ## RESET FLAGS
        CIRCLE_EXIST = 0
        
    webcam.release()
    cv2.destroyAllWindows()
    print(f"ERROR RATE: {error / (error + detect)}")
    
    # # Disable Dynamixel Torque
    # for id in ID:
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))

    # # Close port
    # portHandler.closePort()
    