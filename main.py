from tqdm import tqdm
import time
from core.motorDegreeCalculator import MotorDegreeCalculator as MotorDegreeCalculator
import random
from utils.functions import *
from utils import LowPassFilter, ErrorRecoder
import cv2
from cvDetection.ballDeteectUtil import *
from DynanixelController import DynamixelController

calculator = MotorDegreeCalculator()
lpf = LowPassFilter(100)

### OTHER CONSTANTS
# (0,002, 0,0002) -> 원운동
Kp = 0.00329#0.0091#0.0107
Kd = 0.000698#0.000495#0.00095/2
Ki = 0.441
Kp_ctrl_size = 0.0001
Ki_ctrl_size = 0.01
Kd_ctrl_size = 0.00001

target_coord = (160, 130)
def get_clicked_point(event, x, y, flags, param):
    
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Clicked at (x={}, y={})".format(x, y))
def getBallCenterCoordFromVideo(frame):
    """get ball center coordinate from vidio frame input

    Args:
        frame (??): video frame from the videoCapture

    Returns:
        ball_center (tuple?): 2 dim coordinate of the center of the ball coordinate. If no ball in the frame, returns (-1, -1)
        simple_img (np.array): image vector
    """
    
    ball_center = (-1, -1)
    
    img = np.flip(np.flip(np.array(frame), axis=0), axis=1)
            
    img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    threshold = 60
    filter_img = cv2.inRange(img, (0, 0, 0), (threshold, threshold, threshold))
    #filter_img = cv2.inRange(img, (130, 130, 130), (255, 255, 255))
    
    # 경계선 가져오기
    contours_simple = get_contours(filter_img, 50, True)
    #cv2.imshow("test", filter_img)
    ######################
    # 텍스트 출력하고 경계선 그리기(simple)
    
    
    simple_img = img.copy()
    for cnt in contours_simple:
        
        #cv2.drawContours(simple_img, cnt, -1, BLUE, 5)
        if is_circle(cnt):
            # 원형일 경우 빨간색으로 그리기
            
            ball_center = draw_points(simple_img, cnt, 0.01, (0, 0, 255))                      
            cv2.circle(simple_img, (int(ball_center[0]), int(ball_center[1])), 3, (255, 255, 255), -1)
            
    
    
    return ball_center, simple_img


def setTargetCoordinate(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("New Target Pos (x={}, y={})".format(x, y))
        global target_coord
        target_coord = (x, y)
    return 
    is_target_done = 0
    
    while not is_target_done:
        
        _target_coord =list(map(int, input().split()))
        if (len(_target_coord) == 1):
            is_target_done = 1
        else:
            target_coord = _target_coord
    
    return target_coord



    
def isBallExist(ball_center:tuple):
    return ball_center[0]!=-1 and ball_center[1] != -1



def prettyPrint(error_vector, velocity_vector, target_normal_vector):
    print("WeightedErrVec: ({:.2f}, {:.2f})".format(error_vector[0]*Kp_, error_vector[1]*Kp_), end=' ')
    print("WeightedVeloVec: ({:.2f}, {:.2f})".format(velocity_vector[0]*Kd, velocity_vector[1]*Kd), end=' ')
    print("speed {:.2f}".format(getNorm((velocity_vector[0]*Kd, velocity_vector[1]*Kd))), end=' ')
    print("normalVec: ({:.2f}, {:.2f}, 1.00)".format(target_normal_vector[0], target_normal_vector[1]))
    

# main 함수 부분

if __name__ == '__main__':
    ########## INITISLIZATION ##########
    velocity_manager = ErrorRecoder(10)    
    
    ## WebCam Initialize
    webcam = cv2.VideoCapture(cv2.CAP_DSHOW+1)
    if not webcam.isOpened():
        print("Could not open webcam")
        exit()
    cv2.namedWindow("Webcam")
    cv2.setMouseCallback("Webcam", setTargetCoordinate)
    
    
    
    ## Dynamixel Initialize
    dynamixel_controller = DynamixelController([512, 512, 512])
    setHori = 0
    while webcam.isOpened():
        t0 = time.time()
           
        ### read webcam and get coordinate of ball center
        status, frame = webcam.read() 
        if not status:
            continue
        
        
        ball_center, simple_img = getBallCenterCoordFromVideo(frame)   
        cv2.circle(simple_img, (int(target_coord[0]), int(target_coord[1])), 3, GREEN, -1) ## add dot to target coordinate 
           
        
        
        ### Update target normal vector if ball is detected, else set to (0, 0, 1)
        if (isBallExist(ball_center)):
            ## calculate error vector
            error_vector = np.array([target_coord[0]-ball_center[0], -(target_coord[1]-ball_center[1])] )  
            
            error_vector = lpf.run(error_vector) 
            velocity_vector = velocity_manager.getVelocity(error_vector)
            inte = velocity_manager.getInte(error_vector)
            
            v_norm= getNorm(velocity_vector)
            norm = getNorm(error_vector)
            
            Kp_ = Kp*(min(norm, 30)/norm)
            Kd_ = Kd
            
            #cv2.circle(simple_img, (int(target_coord[0]), int(target_coord[1])), 10, (0, 255, 0), 1)
            if (norm < 100):
                Kp_ /= 1
            elif (norm < 10):
                Kp_ /= 6
                if (v_norm < 80):
                    Kd_ = 0
                
                    
            
                    
                
            
            #print((error_vector[0]*Kp_, error_vector[1]*Kp_))
            #Kp_ = Kp
            
            
            error_vector[0] = error_vector[0]//2*2
            error_vector[1] = error_vector[1]//2*2
            target_normal_vector = ((error_vector[0])*Kp_ - (velocity_vector[0])*Kd_ + inte[0]*Ki, (error_vector[1])*Kp_ - (velocity_vector[1])*Kd_ + inte[1]*Ki, 1)
            
            #print(inte[0]*Ki, inte[1]*Ki)
            
            
            try:
                pass
                #arrow_dist = (int(ball_center[0] - target_normal_vector[0]*300), int(ball_center[1] - target_normal_vector[1]*300))
                #p_arrow = (int(ball_center[0] + error_vector[0]*Kp_*300), int(ball_center[1] - error_vector[1]*Kp_*300))
                #i_arrow = (int(ball_center[0] - velocity_vector[0]*Kd_*300), int(ball_center[1] - velocity_vector[1]*Kd_*300))
                #cv2.arrowedLine(simple_img, (int(ball_center[0]), int(ball_center[1])), arrow_dist, (0,0,255), 2) #red 보리
                #cv2.arrowedLine(simple_img, (int(ball_center[0]), int(ball_center[1])), p_arrow, (255,0,0), 2) # blue 초록
                #cv2.arrowedLine(simple_img, (int(ball_center[0]), int(ball_center[1])), i_arrow, (0,255,0), 2) # green 노링
            except:
                import pdb;pdb.set_trace()
            p_n = getNorm((error_vector[0]*Kp_, error_vector[1]*Kp_))
            d_n = getNorm((velocity_vector[0]*Kd, velocity_vector[1]*Kd))
            if max(p_n, d_n) == p_n:
                simple_text = "MAIN : P"
                simple_img = cv2.putText(simple_img, simple_text, (0, 25), FONT, 1, BLUE)
            else:
                simple_text = "MAIN : D"
                simple_img = cv2.putText(simple_img, simple_text, (0, 25), FONT, 1, GREEN)
            #prettyPrint(error_vector, velocity_vector, target_normal_vector)
            
            
        else:
            target_normal_vector = (0, 0, 1)
        
        
        ### Calculate dynamixel positions from target normal vector
        
        calculator.setTargetNormalVector(target_normal_vector)
        angles = calculator.calculateMotorAngles()
        if (setHori): dy_pos = [512, 512, 512]
        else: dy_pos = [int((angle * 180/math.pi + 150) * 1024/300) for angle in angles]

        
        ### Move Dynamixel to target position
        dynamixel_controller.moveGroupDynamixel(dy_pos)  
        cv2.imshow("Webcam", simple_img)
        

        
        
        ### Input manage
        
        
        
        key = cv2.waitKey(1)
        if (key == -1): continue
        elif key == ord('q'):
            print(f"Final (Kp, Ki, Kd): {(Kp, Ki, Kd)}")
            break
        elif key == ord('a'):
            Kd -= Kd_ctrl_size
            print(f"(Kp, Ki, Kd): {(Kp, Ki, Kd)}")
        elif key == ord('d'):
            Kd += Kd_ctrl_size
            print(f"(Kp, Ki, Kd): {(Kp, Ki, Kd)}")
        elif key == ord('s'):
            Kp -= Kp_ctrl_size
            print(f"(Kp, Ki, Kd): {(Kp, Ki, Kd)}")
        elif key == ord('w'):
            Kp += Kp_ctrl_size
            print(f"(Kp, Ki, Kd): {(Kp, Ki, Kd)}")
        elif key == ord('i'):
            print(f"(Kp, Ki, Kd): {(Kp, Ki, Kd)}")
            Ki += Ki_ctrl_size
        elif key == ord('p'):
            Ki -= Ki_ctrl_size
            print(f"(Kp, Ki, Kd): {(Kp, Ki, Kd)}")
        elif key == ord('+'):
            Kp_ctrl_size *= 10
            Kd_ctrl_size *= 10
            Ki_ctrl_size *= 10
            print(f"ctrl (Kp, Ki, Kd): {(Kp_ctrl_size, Ki_ctrl_size, Kd_ctrl_size)}")
        elif key == ord('-'):
            Kp_ctrl_size *= 0.1
            Kd_ctrl_size *= 0.1
            Ki_ctrl_size *= 0.1
            print(f"ctrl (Kp, Ki, Kd): {(Kp_ctrl_size, Ki_ctrl_size, Kd_ctrl_size)}")
        elif key == ord('b'):
            setHori = 1
        elif key == ord('n'):
            setHori = 0
        else:
            pass
        
        
        
        
    
    webcam.release()
    cv2.destroyAllWindows()
    