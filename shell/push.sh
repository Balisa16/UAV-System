#! /bin/bash
git add .
if [ ! -z "$1" ]
  then
  	str="$*"
    git commit -m "$str"
  else
    git commit -m "Update"
fi
git push origin FP