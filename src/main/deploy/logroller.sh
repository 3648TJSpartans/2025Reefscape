#!/bin/sh
logDir="/u/logs"
archiveDir="/u/archives"
yesterday=$( date -r $(($(date +%s) - $(($(date +%s) % 86400)))) +%s)
yesterDate=$(date -r $(($(date +%s) - 86400)) +"%Y-%m-%d")

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

tar -czf $archiveDir/$yesterDate-logs.tgz oldLogs
rm -rf oldLogs
for file in $logDir/*; do
  if [ -f $file ]; then
    if [ $file != $yesterDate-logs.tgz ]; then
      rm $file
    fi
  fi
done
