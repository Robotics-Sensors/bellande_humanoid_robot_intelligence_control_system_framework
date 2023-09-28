#!/bin/bash

# Git push what is already in the repository
git pull --no-edit; git fetch; git add .; git commit -am "latest pushes"; git push
