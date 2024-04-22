import cv2
import math
import numpy as np

# 사용할 변수들 미리 정의
FONT = cv2.FONT_HERSHEY_DUPLEX
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
FILTER_RATIO = 0.7


# 경계선을 가져오는 함수
def get_contours(img, min_area, is_simple=False):
    # 근사화 방식 Simple : 경계선의 꼭짓점 좌표만 반환
    if is_simple:
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 근사화 방식 None : 모든 경계선을 반환
    else:
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    result = []

    # 경계선 개수만큼 반복
    for cnt in contours:
        # 경계선의 너비가 최소 영역 이상일 때만 result 배열에 추가
        if cv2.contourArea(cnt) > min_area:
            result.append(cnt)

    return result


# 원형인지 여부를 반환하는 함수
def is_circle(cnt):
    cnt_length = cv2.arcLength(cnt, True)
    cnt_area = cv2.contourArea(cnt)

    # ratio가 1에 가까울수록 원형
    ratio = 4 * math.pi * cnt_area / pow(cnt_length, 2)

    if ratio > FILTER_RATIO:
        return True
    else:
        return False


# 꼭짓점을 그리는 함수
def draw_points(img, cnt, epsilon, color):
    try:
        cnt_length = cv2.arcLength(cnt, True)
    except:
        return
    approx = cv2.approxPolyDP(cnt, epsilon * cnt_length, closed=True)
    
    for point in approx:
        
        if color is not None:
            
            
            cv2.circle(img, (point[0][0], point[0][1]), 3, color, -1)
            
    return np.mean(approx, axis=0)[0]





if __name__ == '__main__':
    detect = 0
    error = 0
    
    
    webcam = cv2.VideoCapture(cv2.CAP_DSHOW+1)
    if not webcam.isOpened():
        print("Could not open webcam")
        exit()
        
    while webcam.isOpened():
        status, frame = webcam.read()
        
        if status:
            #img = (np.array(frame) // 64) * 64
            img = np.flip(np.flip(np.array(frame), axis=0), axis=1)
            
            img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            threshold = 60
            filter_img = cv2.inRange(img, (0, 0, 0), (threshold, threshold, threshold))
            #filter_img = cv2.inRange(img, (128, 128, 128), (255, 255, 255))
            target_coord = (img.shape[0]//2, img.shape[1]//2)
            
            
            
            # 경계선 가져오기
            contours_simple = get_contours(filter_img, 50, True)
            #contours_simple = get_contours(img, 50, True)
            #contours_none = get_contours(filter_img, 50, False)
            
            ######################3
            # 텍스트 출력하고 경계선 그리기(simple)
            simple_text = "contours count : " + str(len(contours_simple))
            simple_img = cv2.putText(img.copy(), simple_text, (0, 25), FONT, 1, RED)
            
            for cnt in contours_simple:
                
                cv2.drawContours(simple_img, cnt, -1, BLUE, 5)
                if is_circle(cnt):
                    # 원형일 경우 빨간색으로 그리기
                    ball_center = draw_points(simple_img, cnt, 0.01, RED)
                    err_vec = [target_coord[0]-ball_center[0], target_coord[1]-ball_center[1]]
                    print(err_vec)
                    
                    try:
                        
                        cv2.circle(simple_img, (int(ball_center[0]), int(ball_center[1])), 3, (255, 255, 255), -1)
                        detect += 1
                    except:
                        print("pass")
                        error += 1
                        
                    
                
              
            #######################
            
            
            cv2.imshow("test", simple_img)
            #cv2.imshow("test", filter_img)
            
            
            

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    webcam.release()
    cv2.destroyAllWindows()
    print(f"ERROR RATE: {error / (error + detect)}")
    
    
    
    
    
    
    # 이미지 불러와서 필터링
    img = cv2.imread("shape.jpg")
    filter_img = cv2.inRange(img, (0, 0, 0), (255, 150, 255))

    # 경계선 가져오기
    contours_simple = get_contours(filter_img, 50, True)
    contours_none = get_contours(filter_img, 50, False)

    


    # 이미지 화면에 출력
    cv2.imshow("origin image", img)
    cv2.imshow("filter image", filter_img)
    cv2.imshow("simple image", simple_img)
    cv2.waitKey(0)