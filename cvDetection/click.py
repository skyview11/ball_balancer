import cv2

def get_clicked_point(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Clicked at (x={}, y={})".format(x, y))

# 웹캠 비디오 캡처 객체 생성
cap = cv2.VideoCapture(0)  # 0은 기본 웹캠을 나타냅니다. 여러분의 웹캠이 다른 인덱스를 사용하는 경우 변경하세요.

if not cap.isOpened():
    print("Error: Unable to open webcam.")
    exit()

# 윈도우 생성
cv2.namedWindow("Webcam")
cv2.setMouseCallback("Webcam", get_clicked_point)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame.")
        break

    # 웹캠 프레임 표시
    cv2.imshow("Webcam", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 비디오 캡처 객체 및 창 해제
cap.release()
cv2.destroyAllWindows()