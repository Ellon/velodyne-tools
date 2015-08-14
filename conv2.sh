#! /bin/sh
host=$1
file=$2
scp $host:/tmp/velodyneShot.{,pos.}$file .
`dirname $0`/build/convert velodyneShot.$file velodyneShot.pos.$file shot.$file.pcd
pcl_viewer shot.$file.pcd
