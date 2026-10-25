import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request
import yaml

root=Path('/home/kuang/workspace/chess_robot')
out=root/'runs/camera-io-20261025/preview-regression';out.mkdir(parents=True,exist_ok=True)
config=yaml.safe_load((root/'src/chess_robot_preview/config/channels.yaml').read_text())
config['preview']['bind_address']='127.0.0.1';config['preview']['port']=8098
path=out/'channels.yaml';path.write_text(yaml.safe_dump(config,allow_unicode=True))
log=(out/'node.log').open('w')
proc=subprocess.Popen(['ros2','run','chess_robot_preview','preview_node','--ros-args',
    '-r','__node:=preview_regression_node','-p','config_file:='+str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:
    deadline=time.monotonic()+8
    while True:
        try:
            with opener.open('http://127.0.0.1:8098/api/channels',timeout=1) as response:
                assert len(json.load(response)['channels'])==2
            break
        except Exception:
            if time.monotonic()>deadline:raise
            time.sleep(.1)
    env=os.environ.copy();env.update(PREVIEW_TEST_BASE_URL='http://127.0.0.1:8098',
        PREVIEW_TEST_OUTPUT=str(out),PREVIEW_TEST_SKIP_RESOURCE='1')
    subprocess.run(['python3',str(root/'tools/testing/preview_integration.py')],env=env,check=True,timeout=40)
finally:
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=6)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close()
