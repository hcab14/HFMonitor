#!/bin/bash
# $Id$

function start_server {
cat <<EOF
#!/bin/bash 
sudo ./hfmon_server config_server.xml > log_server.txt 2>&1 &
echo \$! > .pid_server
EOF
}

function  start_client {
cat<<EOF
#!/bin/bash
./hfmon_clientTCPFFT 127.0.0.1 18000 config_FFTProcessor.xml > log_client.txt 2>&1 &
echo \$! > .pid_client
EOF
}

echo -n > .lock_start_$$
if [ $# == 0 ]; then
    status_client=`./status.sh client` 
    status_server=`./status.sh server`
    if [ ${status_client} == RUN ] && [ ${status_server} == RUN ]; then 
	echo OK: client and server are running
    else
	[ ${status_server} != RUN ] && ( $0 server; sleep 30s )
	[ ${status_client} != RUN ] && ( $0 client )
    fi
else
    what=$1
    
    if [ `./status.sh $what` == RUN ]; then
	echo "$what is already running"
	exit 1
    fi
    
    case $what in
	server|client)
	    start_$what > .start_$what.sh
	    chmod u+x .start_$what.sh
	    at now <<EOF
./.start_$what.sh
EOF
	    ;;
	*)
	    echo "unrecognized \"$what\". Allowed services: {server,client}"
	    ;;
    esac
fi
rm -f .lock_start_$$
