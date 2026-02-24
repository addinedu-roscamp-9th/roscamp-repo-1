#!/home/name/server/fastapi-basic-server/venv/bin/python3
"""
간단한 서버 실행 스크립트
./run.py 로 실행하면 서버가 시작됩니다.
기존 서버가 실행 중이면 자동으로 종료하고 새로 시작합니다.
"""
import os
import signal
import subprocess
import time
import sys

try:
    import uvicorn
except ImportError:
    # uvicorn이 없으면 venv 확인
    venv_python = os.path.join(os.path.dirname(os.path.abspath(__file__)), "venv", "bin", "python3")
    if os.path.exists(venv_python):
        print(f"ImportError: uvicorn을 찾을 수 없습니다. 가상환경(venv)으로 전환합니다: {venv_python}")
        # 현재 스크립트를 venv 파이썬으로 다시 실행
        os.execv(venv_python, [venv_python] + sys.argv)
    else:
        print("Error: uvicorn이 설치되어 있지 않고, ./venv 가상환경도 찾을 수 없습니다.")
        print("  pip install -r requirements.txt  명령어로 디펜던시를 설치해주세요.")
        sys.exit(1)

def kill_existing_servers():
    """기존 uvicorn 서버 프로세스를 종료합니다."""
    try:
        # 포트 8000을 사용 중인 프로세스 찾기
        result = subprocess.run(
            ["lsof", "-ti", ":8000"],
            capture_output=True,
            text=True
        )
        
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            
            for pid_str in pids:
                try:
                    pid = int(pid_str.strip())
                    print(f"기존 서버 프로세스 종료 중 (PID: {pid})...")
                    os.kill(pid, signal.SIGKILL)
                except (ValueError, ProcessLookupError):
                    pass  # 이미 종료된 프로세스
                except PermissionError:
                    print(f"경고: PID {pid} 종료 권한이 없습니다.")
            
            time.sleep(1)  # 포트가 완전히 해제될 때까지 대기
            print("기존 서버 종료 완료")
        else:
            print("실행 중인 서버가 없습니다.")
    except FileNotFoundError:
        # lsof가 설치되지 않은 경우 pgrep 사용
        try:
            result = subprocess.run(
                ["pgrep", "-f", "python.*app.main"],
                capture_output=True,
                text=True
            )
            if result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                for pid_str in pids:
                    try:
                        pid = int(pid_str.strip())
                        print(f"기존 서버 프로세스 종료 중 (PID: {pid})...")
                        os.kill(pid, signal.SIGKILL)
                    except (ValueError, ProcessLookupError):
                        pass
                time.sleep(1)
                print("기존 서버 종료 완료")
        except Exception as e:
            print(f"프로세스 확인 중 에러: {e}")
    except Exception as e:
        print(f"프로세스 확인 중 에러: {e}")

if __name__ == "__main__":
    # Ensure we are running from the script's directory so .env is loaded correctly
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    kill_existing_servers()
    print("서버 시작 중...")
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
