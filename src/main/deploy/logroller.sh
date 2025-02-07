#!/bin/bash
echo "Setting up..."
logDir="/u/logs"         # directory holding logs
archiveDir="/u/archives" # directory to store archives
echo "Directories set"

today=$(date +%Y-%m-%d)                        # current date
midnight=$(date -d "$today 00:00:00" +%s)      # unix timestamp of midnight today
yesterday=$(date -d "$today -1 day" +%Y-%m-%d) # yesterday's date
threeDaysAgo=$(date -d "$today -3 days" +%s)   # unix timestamp of midnight three days ago
echo "Timestamps and dates obtained"

cd $logDir || exit 1
mkdir -p oldLogs # temporary directory to hold old logs
echo "Temporary directory created"
echo "Ready to go"

echo "Step 1: Clearing old logs"

movedFiles=0
for file in "$logDir"/*; do      # loop through all files in the log directory
  fileAge=$(date -r "$file" +%s) # get unix timestamp of file modification
  if [[ -f "$file" ]]; then      # check if the file is a regular file, not a directory
    echo -n "Checking file $file... "
    if [[ "$fileAge" -lt "$midnight" ]]; then # check if the file is older than today
      mv "$file" oldLogs                      # if so, move it to the temporary directory
      movedFiles=$((movedFiles + 1))
      echo "moved"
    else
      echo "skipped"
    fi
  fi
done
echo "Step 1 complete: $movedFiles files moved"
if [[ -z "$(ls -A oldLogs)" ]]; then # check if the temporary directory is empty (no files moved)
  rmdir oldLogs
  echo "No files to archive, nothing left to do"
  exit 0 # exit the script to avoid performing unnecessary steps
fi
echo "Step 2: Archiving old logs"
tar -czf $archiveDir/"$yesterday"-logs.tgz oldLogs # tarzips the temporary directory into the archive directory
echo "Removing temporary directory"
rm -rf oldLogs # remove the temporary directory
echo "Step 2 complete"
echo "Step 3: Removing old archives"
remArchives=0
for file in "$archiveDir"/*.tgz; do # loop through all files in the archive directory
  fileAge=$(date -r "$file" +%s)
  if [[ -f "$file" ]]; then                       # check if the file is a regular file, not a directory
    if [[ "$fileAge" -lt "$threeDaysAgo" ]]; then # check if the file is older than three days
      remArchives=$((remArchives + 1))
      rm "$file" # if so, remove the file
    fi
  fi
done
echo "Step 3 complete: $remArchives files removed"
echo "All done"
exit 0
