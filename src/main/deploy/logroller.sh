#!/bin/bash
echo "Setting up..."
logDir="/u/logs"
archiveDir="/u/archives"
echo "Directories set"

today=$(date +%Y-%m-%d)
midnight=$(date -d "$today 00:00:00" +%s)
yesterday=$(date -d "$today -1 day" +%Y-%m-%d)
threeDaysAgo=$(date -d "$today -3 days" +%s)
echo "Timestamps and dates obtained"

cd $logDir || exit 1
mkdir -p oldLogs
echo "Temporary directory created"
echo "Ready to go"

echo "Step 1: Clearing old logs"

movedFiles=0
for file in "$logDir"/*; do
  fileAge=$(date -r "$file" +%s)
  if [ -f "$file" ]; then
  echo -n "Checking file $file... "
   if [ "$fileAge" -lt "$midnight" ]; then
    mv "$file" oldLogs
    movedFiles=$((movedFiles + 1))
    echo "moved"
    else
    echo "skipped"
   fi
  fi
done
echo "Step 1 complete: $movedFiles files moved"
if [ -z "$(ls -A oldLogs)" ]; then
  rmdir oldLogs
  echo "No files to archive, nothing left to do"
  exit 0
fi
echo "Step 2: Archiving old logs"
tar -czf $archiveDir/"$yesterday"-logs.tgz oldLogs
echo "Removing temporary directory"
rm -rf oldLogs
echo "Step 2 complete"
echo "Step 3: Removing old archives"
remArchives=0
for file in "$archiveDir"/*.tgz; do
fileAge=$(date -r "$file" +%s)
  if [ -f "$file" ]; then
    if [ "$fileAge" -lt "$threeDaysAgo" ]; then
      remArchives=$((remArchives + 1))
      rm "$file"
    fi
  fi
done
echo "Step 3 complete: $remArchives files removed"
echo "All done"
exit 0