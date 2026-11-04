"""供 ros2 run 调用的独立测试入口。"""
import argparse
from pathlib import Path


def main(args=None):
    import pytest
    from ament_index_python.packages import get_package_share_directory

    tests = Path(get_package_share_directory("rcs_ptl")) / 'test'
    choices = sorted(p.stem.removeprefix('test_') for p in tests.glob('test_*.py'))
    parser = argparse.ArgumentParser(description='运行全部测试，或指定一个测试组')
    parser.add_argument('group', nargs='?', choices=choices)
    options = parser.parse_args(args)
    target = tests / f'test_{options.group}.py' if options.group else tests
    # 保留 pytest 的退出码，失败时 ros2 run 同样返回非零状态。
    return int(pytest.main([str(target), '-v', '-p', 'no:cacheprovider']))
