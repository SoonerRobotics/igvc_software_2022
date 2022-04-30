#!/bin/bash

bash common.sh

# copy the service to the systemd directory
cp etc/igvc.service /etc/systemd/system/
cp etc/igvc_service.sh /usr/bin/

chmod +x /usr/bin/igvc_service.sh
chmod 644 /etc/systemd/system/igvc.service

# run on boot
# systemctl enable igvc
echo "You can now run 'systemctl enable igvc' to enable the codebase on startup
