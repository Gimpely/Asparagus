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

echo_and_run cd "/home/student/Asparagus_project/ros_proj/src/geometry2/tf2_kdl"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/student/Asparagus_project/ros_proj/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/student/Asparagus_project/ros_proj/install/lib/python3/dist-packages:/home/student/Asparagus_project/ros_proj/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/student/Asparagus_project/ros_proj/build" \
    "/home/student/anaconda3/bin/python3" \
    "/home/student/Asparagus_project/ros_proj/src/geometry2/tf2_kdl/setup.py" \
    egg_info --egg-base /home/student/Asparagus_project/ros_proj/build/geometry2/tf2_kdl \
    build --build-base "/home/student/Asparagus_project/ros_proj/build/geometry2/tf2_kdl" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/student/Asparagus_project/ros_proj/install" --install-scripts="/home/student/Asparagus_project/ros_proj/install/bin"
