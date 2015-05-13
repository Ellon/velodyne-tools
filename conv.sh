#! /bin/sh
host=$1
file=$2
scp $host:/tmp/shot.{,pos.}$file .
`dirname $0`/build/convert shot.$file shot.pos.$file shot.$file.pcd
pcl_viewer shot.$file.pcd
