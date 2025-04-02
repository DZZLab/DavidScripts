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

printf "Argument 1 is %s\n" "$arg1"
printf "Argument 2 is %s\n" "$arg2"


DOCKER_BUILDKIT=1 depot build --platform linux/${platform}64 --ssh default -t 178669/agx_dev:$tag -f $dockerfile . --push