#!/bin/bash
# filepath: /Users/banksmalia/Projects/Robotics/2025Reefscape/src/main/deploy/logroller.sh
echo "Setting up..."
logDir="/u/logs"         # directory holding logs
archiveDir="/u/archives" # directory to store archives
echo "Directories set"

# POSIX-compliant date calculations
TZ=UTC0 # Set timezone to UTC to avoid DST issues
export TZ
now=$(date +%s) # Current Unix timestamp

# Calculate yesterday's date
yesterday_epoch=$(( now - 86400 )) # Subtract 24 hours (86400 seconds)
yesterday=$(date -u -d @$yesterday_epoch +%Y-%m-%d)

# Calculate three days ago
three_days_ago_epoch=$(( now - 259200 )) # Subtract 72 hours (259200 seconds)
threeDaysAgo=$(date -u -d @$three_days_ago_epoch +%s)

# Calculate midnight (start of today) using arithmetic (no re-parsing)
midnight=$(( (now / 86400) * 86400 ))

echo "Timestamps and dates obtained"

cd "$logDir" || exit 1
mkdir -p oldLogs # temporary directory to hold old logs
echo "Temporary directory created"
echo "Ready to go"

echo "Step 1: Clearing old logs"

movedFiles=0
for file in "$logDir"/*; do      # loop through all files in the log directory
  if [[ -f "$file" ]]; then      # check if the file is a regular file, not a directory
    fileAge=$(date -r "$file" +%s) # get unix timestamp of file modification
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
tar -czf "$archiveDir/$yesterday"-logs.tgz oldLogs # tarzips the temporary directory into the archive directory
echo "Removing temporary directory"
rm -rf oldLogs # remove the temporary directory
echo "Step 2 complete"
echo "Step 3: Removing old archives"
remArchives=0
for file in "$archiveDir"/*.tgz; do # loop through all files in the archive directory
  if [[ -f "$file" ]]; then
    archiveName=$(basename "$file")
    archiveDate=${archiveName%-logs.tgz}
    archiveTimestamp=$(date -j -f "%Y-%m-%d" "$archiveDate" +%s 2>/dev/null)

    if [[ -n "$archiveTimestamp" ]] && [[ "$archiveTimestamp" -lt "$threeDaysAgo" ]]; then
      remArchives=$((remArchives + 1))
      rm "$file"
      echo "Removed old archive: $file"
    fi
  fi
done
echo "Step 3 complete: $remArchives files removed"
echo "All done"
exit 0
