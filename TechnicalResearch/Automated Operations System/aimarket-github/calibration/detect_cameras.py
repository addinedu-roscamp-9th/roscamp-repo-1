#!/usr/bin/env python3
"""
카메라 감지 및 정보 확인 스크립트
시스템에 연결된 모든 카메라를 감지하고 정보를 출력합니다.
"""
import cv2
import sys

def detect_cameras(max_cameras=10):
    """
    시스템에 연결된 카메라를 감지합니다.
    
    Args:
        max_cameras: 검사할 최대 카메라 인덱스
        
    Returns:
        list: 사용 가능한 카메라 인덱스 리스트
    """
    available_cameras = []
    
    print("🔍 카메라 검색 중...")
    print("=" * 60)
    
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # 카메라 정보 가져오기
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            backend = cap.getBackendName()
            
            print(f"✅ 카메라 {i} 발견")
            print(f"   해상도: {width}x{height}")
            print(f"   FPS: {fps}")
            print(f"   백엔드: {backend}")
            print()
            
            available_cameras.append(i)
            cap.release()
        else:
            cap.release()
    
    print("=" * 60)
    print(f"총 {len(available_cameras)}대의 카메라 발견: {available_cameras}")
    
    return available_cameras


def test_camera_capture(camera_index):
    """
    특정 카메라에서 프레임을 캡처하여 테스트합니다.
    
    Args:
        camera_index: 테스트할 카메라 인덱스
    """
    print(f"\n📷 카메라 {camera_index} 테스트 중...")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ 카메라 {camera_index}를 열 수 없습니다.")
        return
    
    print("✅ 카메라 열기 성공")
    print("   's' 키를 눌러 스크린샷 저장")
    print("   'q' 키를 눌러 종료")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("❌ 프레임을 읽을 수 없습니다.")
            break
        
        # 정보 오버레이
        text = f"Camera {camera_index} - Press 's' to save, 'q' to quit"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 0), 2)
        
        cv2.imshow(f'Camera {camera_index} Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("종료")
            break
        elif key == ord('s'):
            filename = f'camera_{camera_index}_test.jpg'
            cv2.imwrite(filename, frame)
            print(f"✅ 스크린샷 저장: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=" * 60)
    print("🎥 카메라 감지 도구")
    print("=" * 60)
    
    # 카메라 감지
    cameras = detect_cameras()
    
    if not cameras:
        print("⚠️ 감지된 카메라가 없습니다.")
        sys.exit(1)
    
    # 사용자에게 테스트할 카메라 선택
    print("\n테스트할 카메라 인덱스를 입력하세요 (또는 Enter로 건너뛰기):")
    user_input = input("> ").strip()
    
    if user_input:
        try:
            camera_idx = int(user_input)
            if camera_idx in cameras:
                test_camera_capture(camera_idx)
            else:
                print(f"❌ 카메라 {camera_idx}는 사용 불가능합니다.")
        except ValueError:
            print("❌ 유효하지 않은 입력입니다.")
