#!/bin/bash
# $Id$

function start_server {
    cat <<EOF
#!/bin/bash 
sudo ./bin/perseus_server -c config/perseus_server_NTZ.xml > log_server.txt 2>&1 &
echo \$! > .pid_server
EOF
}

function  start_client {
    cat<<EOF
#!/bin/bash
client_FFTtoFile -c config/FFTProcessor_NTZ.xml log_client.txt 2>&1 &
echo \$! > .pid_client
EOF
}

LOCK_FILE=/tmp/HFMonitor_new.lockfile
(   
    flock -n -e 200 || { echo "This script is currently being run"; exit 1; } >&2

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

) 200>${LOCK_FILE}
