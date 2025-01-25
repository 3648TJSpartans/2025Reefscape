#!/bin/bash
# A log manager script for the roborio
# When run, will loop through each file in /u/logs and check their age
# Files older than 1 day will be archived and zipped, files older than 2 days will be ignored

# Calculate age thresholds in seconds
one_day_old=$(date -d "1 day ago" +%s)
two_day_old=$(date -d "2 days ago" +%s)
date=$(date)

# Setup directories and pwd
cd /u
mkdir -p old-logs

# Loop through each file in the source directory
for file in /u/logs /*; do
  # Check if it's a file (not a directory)
  if [[ -f "$file" ]]; then
    # Get file modification time in seconds
    file_age=$(stat -c %Y "$file")

    # Ignore files younger than 1 day
    if [[ "$file_age" -gt "$one_day_old" ]]; then
      continue
    fi

    # Check if file is older than 1 days
    if [[ "$file_age" -le "$one_day_old" ]]; then
      # Move file to old-logs directory
      mv "$file" /u/old-logs
    fi
  fi
done

# Tar the old directory
tar -czf "$date-logs.tar" old-logs
gzip "$date-logs.tar"
rm "$date-old-logs.tar" # Test if this is deleted beforehand (if so this is uneeded)
rm -r old-logs

# Remove any old log archives
for file in /u/*.tar.gz; do
  file_age=$(stat -c %Y "$file")
  if [[ "$file_age" -le "$two_day_old" ]]; then
    rm "$file"
  fi
done