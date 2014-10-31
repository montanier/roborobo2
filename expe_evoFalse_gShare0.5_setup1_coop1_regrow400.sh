#!/bin/bash

dir="/home/jm/results/dispersion/setup-v2/evoFalse_gShare0.5_setup1_coop1_regrow400"
nbExp=46
nbInst=4

oneRun()
{
	#init variables
	mainTimeStamp=$1

	isSuccess=0
	timeStamp=`date +%H%M%S%N`	

	#launch experiment
	./roborobo -l config/energy-medea.properties 2>"temp"$timeStamp"Error" >"temp$timeStamp"

	#apply post-experiment processing if the last experiment isn't already done
	if [ ! -e "./lastRunDone" ]
	then

		dist=$dir
		logFileName=`grep -a "\[WARNING\] No default gLogFilename string value. Log data will be written in" "temp$timeStamp" | sed -e "s/\[WARNING\] No default gLogFilename string value. Log data will be written in //" | sed -e "s/\"//g"` 

		param=${logFileName#*_}
		mv logs/properties_$param $dist
		mv logs/$logFileName $dist

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
		rm "take$mainTimeStamp" 
		echo "Done : $count/$nbExp"

		rm "temp"$timeStamp"Error"
		rm "temp"$timeStamp

	else
		rm "take$mainTimeStamp" 
		rm "temp$timeStamp"
	fi
}


perl -pi -e 's/gPhysicalObjectDefaultRegrowTimeMax.*/gPhysicalObjectDefaultRegrowTimeMax = 400/' config/energy-medea.properties
perl -pi -e 's/gAltruismEvolved.*/gAltruismEvolved = false/' config/energy-medea.properties
perl -pi -e 's/gSharing.*/gSharing = 0.5/' config/energy-medea.properties
perl -pi -e 's/gSetup.*/gSetup = 1/' config/energy-medea.properties
perl -pi -e 's/gCoopPartner.*/gCoopPartner = 1/' config/energy-medea.properties


mainTimeStamp=`date +%H%M%S%N`	
completeCnt=0
totCnt=0
lastRunDone=0

if [ -e "./lastRunDone" ]
then
	rm lastRunDone
fi

if [ ! -e $dir ]
then
	mkdir $dir
fi

echo "0" > "complete$mainTimeStamp"

while [ $completeCnt -lt $nbExp ]
do
	while [ `jobs | wc -l` -ge $nbInst ] # check the number of instance currently running
	do
		jobs >/dev/null
		sleep 5
	done

	while [ -e "take$mainTimeStamp" ]
	do
		sleep 1
	done
	touch "take$mainTimeStamp" 
	completeCnt=$(cat "complete$mainTimeStamp")
	rm "take$mainTimeStamp" 

	if [ $completeCnt -lt $nbExp ]
	then
		oneRun $mainTimeStamp &
		totCnt=$((totCnt+1))
	fi
	sleep 1
done

touch "./lastRunDone"
killall -9 roborobo


#	./runParallal.sh -p "postExperimentMove.sh $dir" 200 MedeaAggregationInit.properties 7 > log25-neutralEnvironment-initAssembled

