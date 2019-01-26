#!/bin/bash

target=${TARGET:-$(echo $1)}

if [ -z "$target" ] ; then
  echo "Consult the ansible_hosts file and specify a host please!"
  exit 1
fi;

ansible-playbook vision.yml $CHECK --ask-sudo-pass  --extra-vars "target=$target" --ask-pass


