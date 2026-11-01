"""Compatibility entry: supervise both independent read-only ROS nodes."""
import sys
from split_supervisor import main

if __name__ == "__main__":
    sys.exit(main())
