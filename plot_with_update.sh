#!/bin/bash
# $Id$
//
// Copyright 2013 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

function emit_plot_cmd {
    cat <<EOF
plot "<awk '/GQD/{a=0} /DHO/{a=1} /NAA/{a=2} /NRK/{a=3} /ICV/{a=4} /GQ2/{a=5} /^XXX/ {if (a>=0) {print \$10}}' mc.log" w d
EOF
}

(
    cat <<EOF
set grid;
pi=atan(1)*4
set yrange [-.1:.1]
EOF
    while [ 1 ]; do
	emit_plot_cmd; sleep 4; echo "." > /dev/stderr;
    done
) | gnuplot