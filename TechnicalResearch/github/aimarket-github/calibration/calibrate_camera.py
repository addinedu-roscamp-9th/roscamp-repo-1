#!/usr/bin/env python3
"""
카메라 캘리브레이션 계산 스크립트
캡처한 체커보드 이미지를 사용하여 카메라 매트릭스와 왜곡 계수를 계산합니다.
"""
import cv2
import numpy as np
import glob
import os
import json
import sys

# 체커보드 설정
CHECKERBOARD_SIZE = (10, 7)  # 내부 코너 개수 (가로, 세로) - 11열x8행
SQUARE_SIZE = 25  # 체커보드 정사각형 크기 (mm)


def calibrate_camera(images_dir, output_dir, camera_name):
    """
    체커보드 이미지를 사용하여 카메라를 캘리브레이션합니다.
    
    Args:
        images_dir: 체커보드 이미지가 있는 디렉토리
        output_dir: 캘리브레이션 결과를 저장할 디렉토리
        camera_name: 카메라 이름
        
    Returns:
        dict: 캘리브레이션 결과
    """
    print(f"\n{'='*60}")
    print(f"🔬 카메라 '{camera_name}' 캘리브레이션 시작")
    print(f"{'='*60}")
    print(f"📁 이미지 디렉토리: {images_dir}")
    print(f"💾 결과 저장 위치: {output_dir}")
    print(f"📐 체커보드 크기: {CHECKERBOARD_SIZE}")
    print(f"📏 정사각형 크기: {SQUARE_SIZE}mm")
    print(f"{'='*60}\n")
    
    os.makedirs(output_dir, exist_ok=True)
    
    # 3D 포인트 준비 (체커보드의 실제 좌표)
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE  # mm 단위로 스케일링
    
    # 3D 포인트와 2D 포인트를 저장할 배열
    objpoints = []  # 3D 포인트 (실제 좌표)
    imgpoints = []  # 2D 포인트 (이미지 좌표)
    
    # 이미지 파일 찾기
    image_files = sorted(glob.glob(os.path.join(images_dir, "*.jpg")))
    
    if not image_files:
        print(f"❌ {images_dir}에서 이미지를 찾을 수 없습니다.")
        return None
    
    print(f"📷 총 {len(image_files)}장의 이미지 발견")
    print("🔍 체커보드 코너 감지 중...\n")
    
    successful_images = 0
    img_shape = None
    
    for idx, image_file in enumerate(image_files, 1):
        img = cv2.imread(image_file)
        if img is None:
            print(f"⚠️ [{idx:02d}] {os.path.basename(image_file)} - 이미지 로드 실패")
            continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]
        
        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)
        
        if ret:
            # 서브픽셀 정확도로 코너 위치 개선
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            successful_images += 1
            
            print(f"✅ [{idx:02d}] {os.path.basename(image_file)} - 코너 감지 성공")
        else:
            print(f"❌ [{idx:02d}] {os.path.basename(image_file)} - 코너 감지 실패")
    
    print(f"\n{'='*60}")
    print(f"📊 감지 결과: {successful_images}/{len(image_files)}장 성공")
    
    if successful_images < 10:
        print("⚠️ 경고: 성공한 이미지가 너무 적습니다. 최소 10장 이상 권장됩니다.")
        return None
    
    print(f"{'='*60}\n")
    print("🧮 캘리브레이션 계산 중...")
    
    # 카메라 캘리브레이션
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )
    
    if not ret:
        print("❌ 캘리브레이션 실패")
        return None
    
    # 재투영 오차 계산
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], 
                                          camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    
    mean_error = total_error / len(objpoints)
    
    print("✅ 캘리브레이션 완료!\n")
    print(f"{'='*60}")
    print("📊 캘리브레이션 결과")
    print(f"{'='*60}")
    print(f"재투영 오차 (RMS): {mean_error:.4f} pixels")
    print(f"\n카메라 매트릭스:")
    print(camera_matrix)
    print(f"\n왜곡 계수:")
    print(dist_coeffs.ravel())
    print(f"{'='*60}\n")
    
    # 결과 저장
    calibration_data = {
        'camera_name': camera_name,
        'camera_matrix': camera_matrix.tolist(),
        'dist_coeffs': dist_coeffs.ravel().tolist(),
        'rms_error': float(mean_error),
        'image_width': img_shape[0],
        'image_height': img_shape[1],
        'successful_images': successful_images,
        'total_images': len(image_files),
        'checkerboard_size': CHECKERBOARD_SIZE,
        'square_size_mm': SQUARE_SIZE
    }
    
    # JSON 저장
    json_path = os.path.join(output_dir, f'calibration_{camera_name}.json')
    with open(json_path, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    print(f"💾 JSON 파일 저장: {json_path}")
    
    # NumPy 형식 저장
    npz_path = os.path.join(output_dir, f'calibration_{camera_name}.npz')
    np.savez(npz_path, 
             camera_matrix=camera_matrix, 
             dist_coeffs=dist_coeffs,
             rvecs=rvecs,
             tvecs=tvecs)
    print(f"💾 NumPy 파일 저장: {npz_path}")
    
    print(f"\n✅ 캘리브레이션 결과가 {output_dir}에 저장되었습니다.\n")
    
    return calibration_data


if __name__ == "__main__":
    print("=" * 60)
    print("🎯 카메라 캘리브레이션 계산 도구")
    print("=" * 60)
    
    # 사용 가능한 카메라 폴더 찾기
    camera_folders = ['camera_1260', 'camera_1645', 'camera_usb', 'camera_12d0']
    available_cameras = []
    
    for folder in camera_folders:
        images_dir = os.path.join(folder, 'images')
        if os.path.exists(images_dir):
            image_count = len(glob.glob(os.path.join(images_dir, '*.jpg')))
            if image_count > 0:
                available_cameras.append((folder, image_count))
    
    if not available_cameras:
        print("\n❌ 캡처된 이미지가 있는 카메라 폴더를 찾을 수 없습니다.")
        print("   먼저 capture_images.py를 실행하여 이미지를 캡처하세요.")
        sys.exit(1)
    
    print("\n사용 가능한 카메라:")
    for folder, count in available_cameras:
        camera_name = folder.replace('camera_', '')
        print(f"  {camera_name}: {count}장의 이미지")
    
    # 카메라 선택
    print("\n캘리브레이션할 카메라를 선택하세요 (1260/1645/usb/12d0):")
    camera_choice = input("> ").strip().lower()
    
    camera_folder = f"camera_{camera_choice}"
    images_dir = os.path.join(camera_folder, 'images')
    results_dir = os.path.join(camera_folder, 'results')
    
    if not os.path.exists(images_dir):
        print(f"❌ {images_dir} 폴더를 찾을 수 없습니다.")
        sys.exit(1)
    
    # 캘리브레이션 수행
    result = calibrate_camera(images_dir, results_dir, camera_choice)
    
    if result:
        print("✅ 캘리브레이션이 성공적으로 완료되었습니다!")
        print(f"   결과 파일: {results_dir}/calibration_{camera_choice}.json")
    else:
        print("❌ 캘리브레이션에 실패했습니다.")
