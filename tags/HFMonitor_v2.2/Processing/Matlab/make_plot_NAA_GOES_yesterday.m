#! /usr/bin/octave -q
%################################################################################
%										#
% Author: 		Dr. M.A. Danielides					#
% Last modified:	03-26-2012						#
% Function:		Execute first ./load_GOES_yesterday.sh at the		#
% =========		shell. Then you may run this script from shell or 	#
%			via octave. The output is a plot of both 		#
%			GOES X-ray data from yesterday and the 24 kHz (Cutler-	#
%			NAA / USA) VHF fieldstrength recorded at Neustrelitz - 	#
%			Germany.						#
%										#
%################################################################################
tic

clear
clf

% ==============
% Build Filename
% ==============
%Path="/media/Data-Disk/.rw/home/linutop/Data/";
Path="/home/linutop/HFMonitor_new/DataNTZ/"
addpath("./Matlab");

load gestern.txt
jahr=load('jahr.txt');
monat=load('monat.txt');
tag=load('tag.txt');

vuosi = num2str(jahr);
kuu   = num2str(monat);
paiva = num2str(tag);

if (monat < 10)
  kuu=['0',num2str(monat)];
endif

if (tag < 10)
  paiva=['0',num2str(tag)];
else
  paiva=num2str(tag);
endif 

name=['/y',vuosi,'-m',kuu,'-d',paiva,'.txt']

Folder1="L2.024000_Navy_NAA";
filename1=[Path Folder1 name]

% ================================
% Load and filter data from source
% ================================

d1=filter_data(read_data(filename1),60);

% ================================
% Load and process GOES data
% ================================

load stunde.txt
load minute.txt
load GOES_short.txt

ll = length(stunde);

daten(1:ll,1) = jahr;
daten(1:ll,2) = monat;
daten(1:ll,3) = tag;
daten(:,4) = stunde;
daten(:,5) = minute;
daten(:,6) = datenum(daten(:,1),daten(:,2),daten(:,3),daten(:,4),daten(:,5));
daten(:,7) = GOES_short;

% =========================
% Plot data and adjust Plot
% =========================

[A,h1,h2] = plotyy (d1.filter_xx_t, d1.filter_xx_s,daten(:,6),daten(:,7), @plot, @semilogy);
%hold on
ylim(A(1),[-80 -50]);
datetick(A(1),"x","HH:MM");
datetick(A(2),"x","HH:MM");
xlim(A(1),[daten(1,6) daten(length(daten(:,1)),6)]);
xlim(A(2),[daten(1,6) daten(length(daten(:,1)),6)]);
DLR=" at DLR Neustrelitz (contact: michael.danielides-at-dlr.de)";
%Datum=datestr(now,'dd-mmm-yyyy HH:MM:SS');
Datum=[vuosi,'-',kuu,'-',paiva]
ueberschrift=[Datum DLR];
title(ueberschrift)
xlabel('Time [UT]')
ylabel(A(1),'Fieldstrength (const*[V/m])')
ylabel(A(2),'GOES 15 X-RAY (0.5-4.0 A) [Watt/m2]')

store=[Datum,'_NNA_GOES.jpg'];
print(store)
print -djpg Plots/your_daily_Plot4NNA+GOES.jpg

disp("RUN O.K.")
toc
