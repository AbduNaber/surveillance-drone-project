#!/bin/bash
set -e

# Directories
PROJECT_ROOT=$(pwd)
EXECS_DIR="$PROJECT_ROOT/execs"

# Create execs directory
mkdir -p "$EXECS_DIR"
echo ">> Created/Checked execs directory at $EXECS_DIR"

# ==========================================
# Functions
# ==========================================

function build_tello_driver {
    echo ">> Building Tello Driver..."
    cd "$PROJECT_ROOT/DJITelloPy/examples"
    # Using --onefile to create a single executable
    pyinstaller --onefile --name tello_driver tello_driver.py
    cp dist/tello_driver "$EXECS_DIR/"
    echo ">> Tello Driver built and copied."
}

function build_mission_planner {
    echo ">> Building Mission Planner..."
    cd "$PROJECT_ROOT/missionPlanner"
    make clean
    make
    cp mission_planner "$EXECS_DIR/"
    echo ">> Mission Planner built and copied."
}

function build_video_streamer {
    echo ">> Building Video Streamer..."
    cd "$PROJECT_ROOT/person-detection"
    make clean
    make
    cp video_streamer "$EXECS_DIR/"
    echo ">> Video Streamer built and copied."
}

function build_yolo_worker {
    echo ">> Building YoloWorker..."
    cd "$PROJECT_ROOT/person-detection"
    # Building generic onefile executable for YoloWorker
    pyinstaller YoloWorker.spec
    cp dist/YoloWorker "$EXECS_DIR/"
    if [ -f "yolov8s.pt" ]; then
        cp yolov8s.pt "$EXECS_DIR/"
        echo ">> yolov8s.pt model copied."
    else
        echo ">> WARNING: yolov8s.pt not found in person-detection. YoloWorker might fail."
    fi
    cp dist/_interal "$EXECS_DIR/"
    echo ">> YoloWorker built and copied."
}

function print_usage {
    echo "Usage: $0 {tello_driver|mission_planner|video_streamer|YoloWorker|all}"
    echo "You can specify multiple targets, e.g.: $0 tello_driver mission_planner"
}

# ==========================================
# Argument Parsing
# ==========================================

if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

for arg in "$@"; do
    case $arg in
        "tello_driver")
            build_tello_driver
            ;;
        "mission_planner")
            build_mission_planner
            ;;
        "video_streamer")
            build_video_streamer
            ;;
        "YoloWorker")
            build_yolo_worker
            ;;
        "all")
            build_tello_driver
            build_mission_planner
            build_video_streamer
            
            ;;
        *)
            echo "Error: Unknown argument '$arg'"
            print_usage
            exit 1
            ;;
    esac
done

echo ">> Build(s) completed successfully!"
ls -l "$EXECS_DIR"
