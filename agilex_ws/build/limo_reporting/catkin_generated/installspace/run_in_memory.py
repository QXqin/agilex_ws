#!/usr/bin/env python3
import os
import sys
import tempfile
import shutil
import subprocess

def main():
    # 获取原始脚本路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    original_script = os.path.join(script_dir, 'pose_reporter.py')
    
    # 创建内存文件系统
    mem_dir = tempfile.mkdtemp(prefix='ros_mem_')
    
    try:
        # 复制脚本到内存文件系统
        mem_script = os.path.join(mem_dir, 'pose_reporter.py')
        shutil.copy(original_script, mem_script)
        os.chmod(mem_script, 0o755)
        
        # 构建命令行参数
        cmd = [mem_script] + sys.argv[1:]
        
        # 执行内存中的脚本
        process = subprocess.Popen(cmd)
        process.wait()
        
        # 返回退出代码
        sys.exit(process.returncode)
    finally:
        # 清理内存文件系统
        shutil.rmtree(mem_dir, ignore_errors=True)

if __name__ == '__main__':
    main()