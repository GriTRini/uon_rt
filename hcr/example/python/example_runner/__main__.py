import sys
from pathlib import Path
import runpy

def main():
    if len(sys.argv) < 2:
        print("Usage: python example_runner.pyz <example_name>")
        print("Use --list to see available examples")
        sys.exit(2)

    # 현재 실행 중인 pyz 위치 = bin/
    pyz_path = Path(sys.argv[0]).resolve()
    bin_dir = pyz_path.parent

    # 실제 예제 파이썬 코드 위치
    py_root = (bin_dir / ".." / "example" / "python").resolve()

    if not py_root.is_dir():
        raise FileNotFoundError(py_root)

    # import 기준 고정
    sys.path.insert(0, str(py_root))
    sys.path.insert(0, str(bin_dir))

    if sys.argv[1] == "--list":
        for d in sorted(py_root.iterdir()):
            if d.is_dir() and not d.name.startswith("__"):
                print(d.name)
        sys.exit(0)

    name = sys.argv[1]
    p = Path(name)

    if p.suffix == ".py":
        # example014_joint_angle_control.py
        script = py_root / p.stem / p.name
    else:
        # example014_joint_angle_control
        script = py_root / p / f"{p.name}.py"
        script = py_root / name / f"{name}.py"

    if not script.is_file():
        raise FileNotFoundError(script)

    sys.argv = [str(script)] + sys.argv[2:]
    runpy.run_path(str(script), run_name="__main__")

if __name__ == "__main__":
    main()