#!/bin/sh

target=README.md
usr=`pwd | sed 's%^.*/\(.*\)%\1%'`
if [ ! -f $target ];then
    id=0
    echo Create target:$target
    echo $id >> $target
    echo $id:$usr >> $target 
else
    echo "Update target $target"
    id=`head -1 $target`
    id="$[id+1]"
    echo $id > tmp.id
    sed -i -e '1,1d' $target
    cat tmp.id $target > tmp
    mv tmp $target
    echo $id:$usr >> $target 
    rm tmp.id
    cat $target
fi
   


