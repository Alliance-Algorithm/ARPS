git reset .
git add .
if [ $1 ]
then 
    git commit -m "$1 in "$(date +%F)" "$(date +%R)
else
    git commit -m "Update in "$(date +%F)" "$(date +%R)
fi
git push --progress origin main
