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
daten(:,4) = stunde;
daten(:,5) = minute;
daten(:,6) = datenum(daten(:,1),daten(:,2),daten(:,3),daten(:,4),daten(:,5));
daten(:,7) = GOES_short;

%[A,h1,h2] = plotyy (daten(:,6),daten(:,7),daten(:,6),daten(:,7), @plot, @semilogy);
%hold on
%ylim(A(1),[-80 -50]);
%datetick(A(1),"x","HH:MM");
%datetick(A(2),"x","HH:MM");
%DLR=" at DLR Neustrelitz (contact: michael.danielides-at-dlr.de)";
%Datum=datestr(now,'dd-mmm-yyyy HH:MM:SS');
%ueberschrift=[Datum DLR];
%title(ueberschrift)
%xlabel('Time [UT]')
%ylabel(A(1),'Fieldstrength (const*[V/m])')
%ylabel(A(2),'GOES 15 X-RAY (0.5-4.0 A) [Watt/m^2]')

semilogy(daten(:,6),daten(:,7))
ylim([1E-9 1E-2])
grid on
datetick("x","HH:MM");

disp("RUN O.K.")
toc
