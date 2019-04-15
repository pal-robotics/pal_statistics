#!/bin/bash

# start carbon
carbon-cache --config=/etc/carbon/carbon.conf start

# start collectd
service collectd start

# start apache
service apache2 start

# start grafana
service grafana-server start

# if any given commands are specified run them, sleep otherwise
if [ "$#" -gt "0" ]; then
    exec "$@"
else
    sleep infinity
fi
