#!/bin/bash
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)

root=~/git/projects/OrbitESC
joblevel=8

# Don't do this if we have uncommitted changes
if [[ -n $(git status -s) ]]; then
  echo "${yellow}You have uncommitted changes. Please commit or stash them before running this script.${reset}"
  exit 1
fi

# Root submodules
echo "${green}Updating core OrbitESC submodules${reset}"
git submodule update --init --jobs $joblevel

# Initialize submodules that also contain submodules
declare -a RecursiveSubmodules=("$root/lib/Aurora" "$root/lib/ChimeraSim" "$root/lib/CommonTools")
for path in "${RecursiveSubmodules[@]}"; do
  echo "${green}Updating $path${reset}"
  cd $path
  git submodule update --init --recursive --jobs $joblevel
done
