#!/bin/bash

# I wan't to use this to run the long ass command:
# DOCKER_BUILDKIT=1 depot build --platform linux/arm64 --ssh default -t 178669/agx_dev:<output_img_tag> -f <dockerfile_name> . --push
# DO NOT USE --load, it takes forever, use --push then pull the image is MUCH faster.

Help()
{
    # Display Help
    echo "This script builds a Docker image using depot with Docker BuildKit."
    echo
    echo "Syntax: $0 [-t|--tag <tag>] [-p|--platform <platform>] [-f|--file <dockerfile>]"
    echo "options:"
    echo "t     tag"
    echo "p     platform arm or amd"
    echo "f     dockerfile name"
    echo
}


while getopts ":t:p:f:h" opt; do
  case $opt in
    t) tag="$OPTARG"
    ;;
    p) platform="$OPTARG"
    ;;
    f) dockerfile="$OPTARG"
    ;;
    h) Help
        exit 0
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
    ;;
  esac
done

# all args need to be given...
if [ -z "$tag" ] || [ -z "$platform" ] || [ -z "$dockerfile" ]; then
    echo "Error: Missing required argument(s)"
    Help
    exit 1
fi

# Check if the tag is one of the ones that might overwrite
if [[ "$tag" == "dev" || "$tag" == "Ubuntu22_deps" ]]; then
  echo "Tag will be overwriting existing 178669/sd_jetson:$tag, are you sure? [Y|N]"
  read -r overwrite
  if [[ "$overwrite" == "n" || "$overwrite" == "N" ]]; then
    echo "Exiting... No Overwrite."
    exit 0  # Exit the script if the user chooses "n" or "N"
  fi
fi

printf "tag is %s\n" "$tag"
printf "platform is %s\n" "${platform}64"
printf "dockerfile is %s\n" "${dockerfile}"


DOCKER_BUILDKIT=1 depot build --platform linux/${platform}64 --ssh default -t 178669/sd_jetson:$tag -f $dockerfile . --push