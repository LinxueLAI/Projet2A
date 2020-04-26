import psutil

def processinfo(processName):
    pids = psutil.pids()
    for pid in pids:
        # print(pid)
        p = psutil.Process(pid)
        # print(p.name)
        if p.name() == processName:
            print(pid)
            return True  # 如果找到该进程则打印它的PID，返回true
    return False  # 没有找到该进程，返回false


processinfo('turnAround.py')