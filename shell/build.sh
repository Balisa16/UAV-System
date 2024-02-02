#!/usr/bin/bash

if ! grep -q "export EMIRO_PATH=" "$HOME/.bashrc"; then
    	echo "EMIRO_PATH is not defined in .bashrc."
	exit 1
fi

current_dir=$(pwd)
while [ "$current_dir" != "/" ]; do
    directory_path="$current_dir/devel/lib"
    if [ -d "$directory_path" ]; then
        full_path=$(readlink -e "$directory_path")
        # echo "Full path of 'devel/lib': $full_path"
        break
    else
        # Move one level up in the directory hierarchy
        current_dir=$(dirname "$current_dir")
    fi
done

if [ "$current_dir" == "/" ]; then
    echo "Directory 'devel/lib' not found in the current or parent directories."
	exit 1
fi

cd $EMIRO_PATH && catkin build emiro
cp $EMIRO_PATH/release/lib/* $full_path
