#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/navlab-nuc/rover_ros_ws/src/planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/navlab-nuc/rover_ros_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/navlab-nuc/rover_ros_ws/install/lib/python3/dist-packages:/home/navlab-nuc/rover_ros_ws/build/planner/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/navlab-nuc/rover_ros_ws/build/planner" \
    "/usr/bin/python3" \
    "/home/navlab-nuc/rover_ros_ws/src/planner/setup.py" \
     \
    build --build-base "/home/navlab-nuc/rover_ros_ws/build/planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/navlab-nuc/rover_ros_ws/install" --install-scripts="/home/navlab-nuc/rover_ros_ws/install/bin"
