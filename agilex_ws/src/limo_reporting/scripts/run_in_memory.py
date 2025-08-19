#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import tempfile
import shutil
import subprocess

def main():
    # »ñÈ¡Ô­Ê¼½Å±¾Â·¾¶
    script_dir = os.path.dirname(os.path.abspath(__file__))
    original_script = os.path.join(script_dir, 'pose_reporter.py')
    
    # ´´½¨ÄÚ´æÎÄ¼þÏµÍ³
    mem_dir = tempfile.mkdtemp(prefix='ros_mem_')
    
    try:
        # ¸´ÖÆ½Å±¾µ½ÄÚ´æÎÄ¼þÏµÍ³
        mem_script = os.path.join(mem_dir, 'pose_reporter.py')
        shutil.copy(original_script, mem_script)
        os.chmod(mem_script, 0o755)
        
        # ¹¹½¨ÃüÁîÐÐ²ÎÊý
        cmd = [mem_script] + sys.argv[1:]
        
        # Ö´ÐÐÄÚ´æÖÐµÄ½Å±¾
        process = subprocess.Popen(cmd)
        process.wait()
        
        # ·µ»ØÍË³ö´úÂë
        sys.exit(process.returncode)
    finally:
        # ÇåÀíÄÚ´æÎÄ¼þÏµÍ³
        shutil.rmtree(mem_dir, ignore_errors=True)

if __name__ == '__main__':
    main()