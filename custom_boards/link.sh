#!/bin/sh

path=$(dirname $(realpath "$0"))
echo $path
for d in */ ; do
    if [ -e "../boards/$d" ];then
        rm -rf "../boards/$d"
    fi
    cp -r "$path/$d" ../boards/
done
