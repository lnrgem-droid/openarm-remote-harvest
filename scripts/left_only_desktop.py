#!/usr/bin/env python3
"""Desktop console for the isolated left-only release; opening it never moves motors."""
import json
from pathlib import Path
import subprocess


CONFIG = Path.home() / '.local/share/openarm-left-only/desktop.json'


def execute(root, args):
    script = root / 'scripts/left_only_session.py'
    result = subprocess.run(['/usr/bin/python3', str(script), *args], cwd=root)
    if args[0] == 'start' and result.returncode == 3:
        print('\n旧会话需要先停止。请可靠支撑左主臂和左从臂；确认后将先失能，再对齐到共同垂落初始位。')
        if input('两臂已支撑且可重新启动，请输入“已支撑”，其他输入取消：').strip() == '已支撑':
            result = subprocess.run(['/usr/bin/python3', str(script), *args, '--arms-supported'], cwd=root)
    return result


def main():
    root = Path(json.loads(CONFIG.read_text())['release_root'])
    script = root / 'scripts/left_only_session.py'
    if not script.is_file():
        print('单左臂程序目录不存在：' + str(root))
        input('按回车关闭。')
        return 1
    while True:
        # Pick up a newer installed release without retaining yesterday's root.
        root = Path(json.loads(CONFIG.read_text())['release_root'])
        print('\n单左臂遥操 · 笔记本 → Jetson\n'
              '1  启动左主从臂（对齐到共同垂落初始位）\n'
              '2  仅预检（不使能、不运动）\n'
              '3  查看状态\n'
              '4  暂停跟随、请求从臂保持\n'
              '5  支撑后失能并停止\n'
              '0  关闭此窗口\n\n'
              '关闭窗口不会停止机械臂；停止前先支撑左主从臂。')
        try:
            choice = input('选择 [0]：').strip()
            if choice in ('', '0', 'q'):
                return 0
            commands = {'2': ['check'], '3': ['status'], '4': ['hold']}
            if choice == '1':
                print('启动会使左主臂和左从臂缓慢对齐到已保存的垂落初始位，检查通过后进入一致跟随。\n'
                      '请现场确认右臂已隔离、左右接线正确，左主从臂运动范围无人。')
                if input('确认可启动请输入“启动”，其他输入取消：').strip() != '启动':
                    continue
                args = ['start', '--workspace-clear']
            elif choice == '5':
                print('失能后机械臂可能下落，必须先可靠支撑左主臂和左从臂。')
                if input('已支撑请输入“已支撑”，其他输入取消：').strip() != '已支撑':
                    continue
                args = ['disable', '--arms-supported']
            elif choice in commands:
                args = commands[choice]
            else:
                print('请输入 0—5。')
                continue
            result = execute(root, args)
            if result.returncode:
                print('\n操作未完成，请查看上面的原因；不能把报错当作已经失能。')
            input('\n按回车返回菜单。')
        except EOFError:
            return 0
        except KeyboardInterrupt:
            print('\n操作被中断；这不代表机械臂已停止或失能，请核对状态。')


if __name__ == '__main__':
    raise SystemExit(main())
