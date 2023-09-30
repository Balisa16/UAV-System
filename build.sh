cd ../..
BASEDIR=$(dirname $0)
echo "Build : $BASEDIR"
catkin build emiro
cd $BASEDIR