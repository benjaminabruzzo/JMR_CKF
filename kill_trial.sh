#!/bin/bash
# host=$1
kill "$(< ~/ros/${HOST}_trials_pid.txt)"
killall screen
