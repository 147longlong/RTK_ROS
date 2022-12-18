#!/bin/sh
# rtkrcv startup script

curpath=$(dirname $(readlink -f "$0"))
$curpath/str2str -in ntrip://chichengcn:chi671003@112.65.161.226:2101/BRUX0 -out tcpsvr://:8101 -out tcpsvr://:8201 &
$curpath/str2str -in ntrip://chichengcn:chi671003@112.65.161.226:2101/RTCM3EPH-MGEX#rtcm3 -out tcpsvr://:8102 -out tcpsvr://:8202 &
echo statup script ok

str2str -in ntrip://Geespace07:PPPRTK1062021@222.188.82.25:8990/Test2 -out serial://pts/19:115200:8:n:1:off