#!/bin/bash

# Get the URL from .git/config
git_url=$(git config --get remote.origin.url)

# Check if a URL is found
if [ -z "$git_url" ]; then
  echo "No remote URL found in .git/config."
  exit 1
fi

# Clone the repository into a temporary folder
git clone "$git_url" tmp_clone

# Check if the clone was successful
if [ $? -eq 0 ]; then
  # Remove the existing .git directory if it exists
  if [ -d ".git" ]; then
    rm -rf .git
  fi

  # Copy the .git directory from the clone to the current repository
  cp -r tmp_clone/.git .

  # Remove the clone directory
  rm -rf tmp_clone

  echo "Repository cloned and .git directory copied successfully."
else
  echo "Failed to clone the repository."
fi
