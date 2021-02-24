#!/bin/bash

dest_path=~/projects/gta_data/OptiTrack/20210213/

mkdir -p ${dest_path}

for i in {226..229}
do
   cp /media/ral2020/SP\ PHD\ U3/Session\ 2020-12-21\ processed/SCH_Trial_${i}.csv ${dest_path}
done

matlab -nodisplay -nosplash -nodesktop -r "cd ${dest_path}..; convert_csv_to_mat('${dest_path}');exit;"
