#! /bin/bash

git add .
if [ $1 ]
then 
    git commit -m "$1"
else
    git commit -m "Update in "$(date +%F)" "$(date +%R)
fi
git push origin main --progress
