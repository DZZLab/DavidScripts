#!/bin/bash
eval "$(ssh-agent -s)"
sshKey="$(cat ~/.ssh/id_rsa 2>/dev/null)"

noSSHans="N"

if [[ -z "${sshKey}" ]]; then
  echo -e "\e[31mNo SSH key found, generate one? [Y/N]\e[0m"
  read -r noSSHans
else
  echo -e "\e[32mSSH key found, moving to Git config\e[0m"
  ssh-add ~/.ssh/id_rsa
fi

if [[ "${noSSHans}" == "Y" || "${noSSHans}" == "y" ]]; then
  echo "Enter email to generate SSH Key with:"
  read -r sshEmail
  ssh-keygen -t rsa -b 4096 -C "$sshEmail"
  ssh-add ~/.ssh/id_rsa
  echo -e "\e[31mKey generated, still need to copy key and add to GitHub\e[0m"
fi

echo "Would you like to configure global Git user.email and user.name now? [Y/N]"
read -r configGit

if [[ "${configGit}" == "Y" || "${configGit}" == "y" ]]; then
  email="$(git config --global user.email)"
  name="$(git config --global user.name)"

  while [[ -z "${email}" ]]; do
    echo "No git global email found, set one:"
    read -r gitEmail
    git config --global user.email "$gitEmail"
    email="$(git config --global user.email)"
  done

  while [[ -z "${name}" ]]; do
    echo "No git global name found, set one:"
    read -r gitName
    git config --global user.name "$gitName"
    name="$(git config --global user.name)"
  done
else
  echo -e "\e[33mSkipped Git config setup.\e[0m"
fi
