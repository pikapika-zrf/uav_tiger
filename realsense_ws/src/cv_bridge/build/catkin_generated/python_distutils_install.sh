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

echo_and_run cd "/home/nvidia/realsense_ws/src/cv_bridge"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/cv_bridge340/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/cv_bridge340/lib/python2.7/dist-packages:/home/nvidia/realsense_ws/src/cv_bridge/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nvidia/realsense_ws/src/cv_bridge/build" \
    "/usr/bin/python2" \
    "/home/nvidia/realsense_ws/src/cv_bridge/setup.py" \
    egg_info --egg-base /home/nvidia/realsense_ws/src/cv_bridge/build \
    build --build-base "/home/nvidia/realsense_ws/src/cv_bridge/build" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local/cv_bridge340" --install-scripts="/usr/local/cv_bridge340/bin"
