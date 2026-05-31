#!/usr/bin/env python3
"""
step01_verify_installation.py
Isaac Sim 5.1 설치 검증 스크립트
사용법: python step01_verify_installation.py

Phase 1 — Step 01: Isaac Sim 5.1 설치하기
"""

import sys
import platform


def check_python_version():
    """Python 버전 확인 (3.11 필요)"""
    version = sys.version_info
    print(f"[1/4] Python 버전: {sys.version}")

    if version.major == 3 and version.minor == 11:
        print("  ✅ Python 3.11 - 적합")
        return True
    else:
        print(f"  ❌ Python 3.11 필요 (현재: {version.major}.{version.minor})")
        print("  💡 python3.11 -m venv env_isaacsim 로 가상환경을 생성하세요")
        return False


def check_isaacsim_import():
    """Isaac Sim import 확인"""
    print("\n[2/4] Isaac Sim 패키지 확인...")
    try:
        from isaacsim import SimulationApp
        print("  ✅ isaacsim 패키지 import 성공")
        return True
    except ImportError as e:
        print(f"  ❌ Import 실패: {e}")
        print("  💡 pip install 명령으로 isaacsim을 설치했는지 확인하세요:")
        print("     pip install isaacsim[all,extscache]==5.1.0 --extra-index-url https://pypi.nvidia.com")
        return False


def check_headless_creation():
    """Headless SimulationApp 생성 확인"""
    print("\n[3/4] SimulationApp 생성 (headless)...")
    try:
        from isaacsim import SimulationApp

        # Headless 앱 생성
        app = SimulationApp({"headless": True})
        print("  ✅ SimulationApp 생성 성공")

        # 기본 Stage 정보 출력
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage:
            print(f"  ✅ Stage 로드 완료: {stage}")
        else:
            print("  ⚠️  Stage 정보 없음")

        # 정리
        app.close()
        print("  ✅ SimulationApp 종료 완료")
        return True

    except Exception as e:
        print(f"  ❌ SimulationApp 생성 실패: {e}")
        return False


def check_gpu_info():
    """GPU 정보 확인"""
    print("\n[4/4] GPU 정보 확인...")
    try:
        import subprocess

        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,memory.total,driver_version",
                "--format=csv,noheader",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0:
            print("  ✅ GPU 감지됨:")
            for line in result.stdout.strip().split("\n"):
                print(f"     {line}")
            return True
        else:
            print("  ❌ nvidia-smi 실행 실패")
            return False
    except FileNotFoundError:
        print("  ❌ nvidia-smi를 찾을 수 없음 (NVIDIA 드라이버 미설치)")
        return False
    except Exception as e:
        print(f"  ⚠️  GPU 정보 확인 중 오류: {e}")
        return False


def print_system_info():
    """시스템 정보 출력"""
    print(f"시스템: {platform.system()} {platform.release()}")
    print(f"머신: {platform.machine()}")
    print()


def main():
    print("=" * 60)
    print("Isaac Sim 5.1 설치 검증")
    print("=" * 60)
    print_system_info()

    results = []
    results.append(check_python_version())
    results.append(check_isaacsim_import())
    results.append(check_headless_creation())
    results.append(check_gpu_info())

    print("\n" + "=" * 60)
    print("검증 결과 요약")
    print("=" * 60)
    all_pass = all(results)
    for i, r in enumerate(results):
        status = "✅ PASS" if r else "❌ FAIL"
        print(f"  Test {i+1}: {status}")

    if all_pass:
        print("\n🎉 모든 검증 통과! Isaac Sim 5.1 설치 완료!")
        print("다음 명령으로 GUI를 실행하세요: isaacsim")
    else:
        print('\n⚠️  일부 검증이 실패했습니다.')
        print('   docs/01-phase-1-foundation/01-step-installation.md 의')
        print('   "문제 해결 (Troubleshooting)" 섹션을 확인하세요.')

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
