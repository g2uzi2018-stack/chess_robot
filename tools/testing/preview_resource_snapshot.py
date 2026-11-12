import json
from pathlib import Path
import subprocess
import os
import time
result={'monotonic_s':time.monotonic()}
for name,package,executable in [('preview','chess_robot_preview','preview_node'),('camera','chess_robot_vision','vision_node')]:
    pattern='^/home/kuang/workspace/chess_robot/install_vision/'+package+'/lib/'+package+'/'+executable
    pids=subprocess.check_output(['pgrep','-f',pattern],text=True).split()
    if len(pids)!=1:raise RuntimeError('Expected one '+name+' process')
    pid=int(pids[0]);fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
    rss=int(next(s.split()[1] for s in Path(f'/proc/{pid}/status').read_text().splitlines() if s.startswith('VmRSS:')))
    result[name]={'pid':pid,'cpu_s':(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),'rss_mib':rss/1024}
print(json.dumps(result))
