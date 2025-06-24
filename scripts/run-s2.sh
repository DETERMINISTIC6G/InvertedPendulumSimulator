#!/bin/bash

INPUT_BASE_DIR="robots-5g-1-pdc-state"
OUTPUT_BASE_DIR="results-s2-test/5g-1-pdc-state"
PROGRAM="./simulate-agv"


mkdir -p "$OUTPUT_BASE_DIR"

counter=0
start=$(date +%s)

#find "$INPUT_BASE_DIR" -type f -name "*plant*.csv" | grep '_dstt_nwtt' | while read input_file; do
find "$INPUT_BASE_DIR" -type f \( -name "*plant1*.csv" -o -name "*plant2*.csv" \) | grep 'TwoRobots' | while read -r input_file; do
   
    #input_rel_path="${input_file#$INPUT_BASE_DIR/}"
    if [[ "$input_file" == *plant1* ]]; then
        r="r1"
        output_file="$OUTPUT_BASE_DIR/${r}_pid_high.csv"
        $PROGRAM -i "$input_file" -o "$output_file" -n 1 -d -1.0 -e -0.05
    elif [[ "$input_file" == *plant2* ]]; then
        r="r2"
        output_file="$OUTPUT_BASE_DIR/${r}_pid_high.csv"
        $PROGRAM -i "$input_file" -o "$output_file" -n 1 -d 1.0 -e 0.05
    else
        r="unknown"
    fi 

    
    ((counter++))
done

end=$(date +%s)
elapsed=$((end - start))

echo "Processed in $elapsed seconds."