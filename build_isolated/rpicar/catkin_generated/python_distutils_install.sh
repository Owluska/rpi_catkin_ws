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

echo_and_run cd "/home/pi/catkin_ws/src/rpicar"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pi/catkin_ws/install_isolated/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pi/catkin_ws/install_isolated/lib/python2.7/dist-packages:/home/pi/catkin_ws/build_isolated/rpicar/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pi/catkin_ws/build_isolated/rpicar" \
    "/usr/bin/python2" \
    "/home/pi/catkin_ws/src/rpicar/setup.py" \
     \
    build --build-base "/home/pi/catkin_ws/build_isolated/rpicar" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pi/catkin_ws/install_isolated" --install-scripts="/home/pi/catkin_ws/install_isolated/bin"
