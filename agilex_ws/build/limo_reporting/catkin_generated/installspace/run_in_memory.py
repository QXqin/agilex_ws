#!/usr/bin/env python3
import os
import sys
import tempfile
import shutil
import subprocess

def main():
    # ��ȡԭʼ�ű�·��
    script_dir = os.path.dirname(os.path.abspath(__file__))
    original_script = os.path.join(script_dir, 'pose_reporter.py')
    
    # �����ڴ��ļ�ϵͳ
    mem_dir = tempfile.mkdtemp(prefix='ros_mem_')
    
    try:
        # ���ƽű����ڴ��ļ�ϵͳ
        mem_script = os.path.join(mem_dir, 'pose_reporter.py')
        shutil.copy(original_script, mem_script)
        os.chmod(mem_script, 0o755)
        
        # ���������в���
        cmd = [mem_script] + sys.argv[1:]
        
        # ִ���ڴ��еĽű�
        process = subprocess.Popen(cmd)
        process.wait()
        
        # �����˳�����
        sys.exit(process.returncode)
    finally:
        # �����ڴ��ļ�ϵͳ
        shutil.rmtree(mem_dir, ignore_errors=True)

if __name__ == '__main__':
    main()