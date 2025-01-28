#!/bin/sh
# A log manager script for the roborio
# When run, will loop through each file in /u/logs and check their age
# Files older than 1 day will be archived and zipped, files older than 2 days will be ignored

# Calculate age thresholds in seconds
one_day_old=$(date -r $(($(date +%s) - 86400)) +%s)
date=$(date)

# Setup directories and pwd
cd /u/logs
mkdir -p old-logs
echo "Setup complete."
movedcounter=0
# Loop through each file in the source directory
echo "Beginning log rotation... "
for file in /u/logs/*.wpilog; do
  # Check if it's a file (not a directory)
  if [ -f "$file" ]; then
    # Get file modification time in seconds
    file_age=$(stat -f %m "$file")
    echo -n "Processing file $file..."
    # Ignore files younger than 1 day
    if [ "$file_age" -gt "$one_day_old" ]; then
      echo "Left alone"
      continue
    fi

    # Check if file is older than 1 day
    if [ "$file_age" -le "$one_day_old" ]; then
      # Move file to old-logs directory
      mv "$file" /u/old-logs
      echo "Rotated"
      movedcounter=$((movedcounter+1))
    fi
  fi
done
echo "Process complete. Rotated $movedcounter files."
echo "Archiving old logs..."
# Tar the old directory
tar -czf "$date-logs.tar" old-logs
echo "Old logs tarred as $date-logs.tar"
echo "Compressing..."
# gzip the tarball
gzip "$date-logs.tar"
echo "Old logs compressed as $date-logs.tar.gz"
echo "Cleaning up..."
rm "$date-old-logs.tar" # Test if this is deleted beforehand (if so this is unneeded)
rm -r old-logs
echo "Cleanup complete."

# Remove any old log archives
echo "Removing old log archives..."
delcounter=0
for file in /u/logs/*.tar.gz; do
  echo -n "Checking tarball $file..."
  if [ "$file" != "$date-logs.tar.gz" ]; then
    rm "$file"
    echo "Deleted"
    delcounter=$((delcounter+1))
  else
    echo "Left alone"
  fi
done
echo "Cleanup complete. Deleted $delcounter old archives."
echo "Log management complete."