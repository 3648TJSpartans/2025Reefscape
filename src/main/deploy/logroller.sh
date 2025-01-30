#!/bin/sh
logDir="/u/logs"
archiveDir="/u/archives"
today=$(date +%Y-%m-%d)
yesterday=$(echo "$today 00:00:00" | awk '{ split($0, a, "[- :]"); print mktime(a[1] " " a[2] " " a[3] " " a[4] " " a[5] " " a[6]) }')

cd $logDir
mkdir -p oldLogs

for file in $logDir/*; do
  fileAge=$(date -r $file +%s)
  if [ -f $file ]; then
   if [ $fileAge -lt $yesterday ]; then
     mv $file oldLogs
   fi
  fi
done

if [ -z "$(ls -A oldLogs)" ]; then
  rmdir oldLogs
  exit 0
fi

tar -czf $archiveDir/$today-logs.tgz oldLogs
rm -rf oldLogs
for file in $archiveDir/*.tgz; do
  if [ -f $file ]; then
    if [ $file != $today-logs.tgz ]; then
      rm $file
    fi
  fi
done
