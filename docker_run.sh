#!/bin/bash
############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Add description of the script functions here."
   echo
   echo "Syntax: scriptTemplate [-g|h|v|V]"
   echo "options:"
   echo "g     Print the GPL license notification."
   echo "h     Print this Help."
   echo "v     Verbose mode."
   echo "V     Print software version and exit."
   echo
}

############################################################
############################################################
# Main program                                             #
############################################################
############################################################

#!/bin/bash

while getopts ":n:i:" opt; do
  case $opt in
    n) arg1="$OPTARG"
    ;;
    i) arg2="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
    ;;
  esac
done

printf "Argument 1 is %s\n" "$arg1"
printf "Argument 2 is %s\n" "$arg2"

echo "Creating Docker container from image: $arg2, named: $arg1"
echo "docker run --name $arg1 --runtime=nvidia --gpus all --network=host --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -it $arg2"


if [[ $(sudo lshw -C display | grep vendor) =~ Nvidia ]]; then
  Nvidia="--runtime=nvidia --gpus all"
else
  Nvidia=" "
fi


# Run container + mounting SSH-keys. [-f /dev/null] allows for running in background
docker run -it --name $arg1 $Nvidia --network=host --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -v ~/.ssh/:/root/.ssh -v=$HOME/.Xauthority:/root/.Xauthority $arg2


# Run commands within container
#docker exec -it $arg1 bash
#echo "test"
#docker exec -it $arg1 bash -c "eval "$(ssh-agent -s)" && echo "Port 2233" >> /etc/ssh/sshd_config && echo "PasswordAuthentication yes" >> /etc/ssh/sshd_config && echo "X11Forwarding yes" >> /etc/ssh/sshd_config && service ssh restart"
