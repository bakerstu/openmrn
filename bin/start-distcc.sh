#!/bin/bash
#
# Starts ssh with port fordwaring for your remote distCC host.



DISTCC_PORT=$(< ~/.distcc/port)
if [ -z "${DISTCC_PORT}" ] ; then
    echo missing ~/.distcc/port. See DISTCC.md. >&2
    exit 1
fi

DISTCC_RHOST=$(< ~/.distcc/ssh_host)
if [ -z "${DISTCC_RHOST}" ] ; then
    echo missing ~/.distcc/ssh_host. See DISTCC.md. >&2
    exit 1
fi

DISTCC_SSHARGS=$(< ~/.distcc/ssh_args)


set -x

ssh ${DISTCC_SSHARGS} -L localhost:${DISTCC_PORT}:localhost:${DISTCC_PORT} ${DISTCC_RHOST} "killall distccd; ~/bin/distccd --no-detach --listen 127.0.0.1 --jobs 20 -p ${DISTCC_PORT} --allow 127.0.0.1/24 --verbose --daemon --log-stderr"
