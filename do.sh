#! /bin/bash
p=~/datasets/laas_2015_08_12/log_minnie_2015_08_12_11_31_21/velodyne

#for f in $p/velodyneShot.10[0-9][0-9]; do
mkdir log_minnie_2015_08_12_11_31_21
cd log_minnie_2015_08_12_11_31_21

for f in $p/velodyneShot.[0-9]*; do
    i=${f##*.} # get the %04i indice
    ~/velodyne-tools/convert_ascii $f > cloud.$(printf %05i $((10#$i))).txt
done
python ~/velodyne-tools/ascii_bag.py 0 9999 "/mana/velodyne" all.bag
