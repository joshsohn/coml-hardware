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

echo_and_run cd "/home/raphaelpeabody/r/simulation_ws/src/trajectory_generator_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/raphaelpeabody/r/simulation_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/raphaelpeabody/r/simulation_ws/install/lib/python3/dist-packages:/home/raphaelpeabody/r/simulation_ws/build/trajectory_generator_python/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/raphaelpeabody/r/simulation_ws/build/trajectory_generator_python" \
    "/usr/bin/python3" \
    "/home/raphaelpeabody/r/simulation_ws/src/trajectory_generator_python/setup.py" \
     \
    build --build-base "/home/raphaelpeabody/r/simulation_ws/build/trajectory_generator_python" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/raphaelpeabody/r/simulation_ws/install" --install-scripts="/home/raphaelpeabody/r/simulation_ws/install/bin"
