#! /usr/bin/octave -q
%################################################################################
%										#
% Author: 		Dr. M.A. Danielides					#
% Last modified:	03-22-2012						#
% Function:		ENTER TEXT here						#
% =========									#
%										#
%################################################################################
tic

clear
clf
% ==============
% Build Filename
% ==============
%Path="/media/Data-Disk/.rw/home/linutop/Data/";
Path="/home/linutop/HFMonitor/Data/";

Year=datestr(now,'YYYY');
Month=datestr(now,'mm');
%Month="01";
Day=datestr(now,'DD');
%Day="31";
name=['/y',Year,'-m',Month,'-d',Day,'.txt']

Folder1="L2.024000_Navy_NAA";
filename1=[Path Folder1 name];

% ================================
% Load and filter data from source
% ================================

d1=filter_data(read_data(filename1),60);

% ================================
% Load and process GOES data
% ================================

load heute.txt
load jahr.txt
load monat.txt
load tag.txt
load stunde.txt
load minute.txt
load GOES_short.txt

ll = length(stunde);

daten(1:ll,1) = jahr;
daten(1:ll,2) = monat;
daten(1:ll,3) = tag;
%daten(1:ll,3) = 21;
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
%ylim(A(2),[1E-9 1E-2]);
datetick(A(1),"x","HH:MM");
datetick(A(2),"x","HH:MM");
xlim(A(1),[daten(1,6) daten(length(daten(:,1)),6)]);
xlim(A(2),[daten(1,6) daten(length(daten(:,1)),6)]);
DLR=" at DLR Neustrelitz (contact: michael.danielides-at-dlr.de)";
Datum=datestr(now,'dd-mmm-yyyy HH:MM:SS');
ueberschrift=[Datum DLR];
title(ueberschrift)
xlabel('Time [UT]')
ylabel(A(1),'Fieldstrength (const*[V/m])')
ylabel(A(2),'GOES 15 X-RAY (0.5-4.0 A) [Watt/m2]')

%legend("24,0 kHz NAA","GOES X-Ray")

store=[date,'_NNA_GOES.jpg'];
print(store)
print -djpg your_daily_Plot4NNA+GOES.jpg

disp("RUN O.K.")
toc
