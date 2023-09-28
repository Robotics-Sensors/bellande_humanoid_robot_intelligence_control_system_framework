#!/bin/bash

# Git push what is already in the repository
git pull --no-edit; git fetch; git add .; git commit -am "latest pushes"; git push

# Get the current directory
current_dir=$(pwd)

# Read the remote repository URL from .git/config
remote_repo_url=$(git -C "$current_dir" config --get remote.origin.url)

# Create a temporary directory for cloning the repository
temp_dir=$(mktemp -d)

# Clone the repository into the temporary directory without using local references
git clone --no-local "$current_dir" "$temp_dir"

# Switch to the temporary directory
cd "$temp_dir"

# Create a temporary file to store the file list
tmp_file=$(mktemp)
# Create a temporary file to store the processed commits
processed_commits_file=$(mktemp)

# Function to check if a commit has already been processed
is_commit_processed() {
  local commit="$1"

  # Check if the commit is already processed
  grep -Fxq "$commit" "$processed_commits_file"
}

# Function to mark a commit as processed
mark_commit_processed() {
  local commit="$1"

  # Mark the commit as processed
  echo "$commit" >> "$processed_commits_file"
}

# Function to check if a file or folder exists in the repository
file_exists_in_repo() {
  local file_path="$1"

  # Check if the file or folder exists in the repository
  git ls-tree --name-only -r HEAD | grep -Fxq "$file_path"
}

# Function to process the files and folders in each commit
process_commit_files() {
  local commit="$1"

  # Check if the commit has already been processed
  if is_commit_processed "$commit"; then
    echo "Commit $commit already processed. Skipping..."
    return
  fi

  # Get the list of files and folders in the commit (including subfolders)
  git ls-tree --name-only -r "$commit" >> "$tmp_file"

  # Process each file or folder in the commit
  while IFS= read -r line
  do
    # Check if the file or folder exists in the current push
    if file_exists_in_repo "$line"; then
      echo "Keeping: $line"
    else
      echo "Deleting: $line"
      git filter-repo --path "$line" --invert-paths
    fi
  done < "$tmp_file"

  # Mark the commit as processed
  mark_commit_processed "$commit"

  # Clear the temporary file
  > "$tmp_file"
}

# Iterate over each commit in the repository
git rev-list --all | while IFS= read -r commit
do
  process_commit_files "$commit"
done

# Push the filtered changes to the original repository
git remote add origin "$remote_repo_url"
git push --force origin main

# Perform a history rewrite to remove the filtered files
git filter-repo --force

# Fetch the changes from the remote repository
git -C "$current_dir" fetch origin

# Merge the remote changes into the local repository
git -C "$current_dir" merge origin/main --no-edit

# Update the local repository and reduce the size of .git if needed
git -C "$current_dir" gc --prune=now
git -C "$current_dir" reflog expire --expire=now --all
git -C "$current_dir" repack -ad

# Clean up temporary files and directories
cd "$current_dir"
rm -rf "$temp_dir"
rm "$tmp_file"
rm "$processed_commits_file"
