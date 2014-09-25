#!/bin/bash

threshold=$1
destDir=$2

hostname=`hostname`

#take a look at results
logFileName=`grep -a "\[WARNING\] No default gLogFilename string value. Log data will be written in" "temp$timeStamp" | sed -e "s/\[WARNING\] No default gLogFilename string value. Log data will be written in //" | sed -e "s/\"//g"` 
parameter=${logFileName#datalog_*}
parameter=${parameter%*.txt}
echo $parameter
logFileName="logs/"$logFileName
echo $logFileName

if [ `perl -ne 'print "$.\n" if /\x00/;' $logFileName | wc -l` -gt 0 ]
then 
	echo "NULL character in $logFileName. This file won't be taken into account." 
else
	activeCount=`grep -a " : pop_alive" $logFileName | tail -n 1 | sed -r -e "s/[0-9]+ : pop_alive //"`

	#transmit log file if necessary
	if (( $activeCount > $threshold ))
	then
		#wait until can update the results
		while [ -e "take$mainTimeStamp" ]
		do
			sleep 1
		done
		#update results
		touch "take$mainTimeStamp" 
		count=$(cat	"complete$mainTimeStamp")
		count=$((count+1))
		echo $count > "complete$mainTimeStamp"

		../Scripts/extract/extractActiveCount.py -F $logFileName
		../Scripts/extract/extractGenomeListSize.py -F $logFileName
		../Scripts/extract/extractNbContact.py -F $logFileName
		../Scripts/extract/extractRelatedness.py -F $logFileName
		../Scripts/extract/extractDistance.py -F $logFileName

		rm "take$mainTimeStamp" 
		rm $logFileName 

		mv "logs/extract_"*".csv" $destDir
		mv "logs/properties_$parameter.txt" $destDir
		mv "logs/trajectory_"$parameter"_"*".bmp" $destDir
	else
		echo "$hostname Extinction"
		rm $logFileName
		rm "logs/properties_$parameter"
	fi
	echo "$hostname Done : $count/$nbExp"
fi
rm "temp$timeStamp"
rm "temp"$timeStamp"Error"
