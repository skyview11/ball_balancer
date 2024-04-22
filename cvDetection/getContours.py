import cv2

def get_contours(img, min_area=100, is_simple=True):
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