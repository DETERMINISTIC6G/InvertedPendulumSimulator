#!/bin/bash

INPUT_BASE_DIR="5g-1-pdc-cycle20ms"
OUTPUT_BASE_DIR="results-s1-test/5g-1-pdc-cycle20ms/lqr-angle-dn"
PROGRAM="./simulate-event_queue"


mkdir -p "$OUTPUT_BASE_DIR"

counter=0
start=$(date +%s)

find "$INPUT_BASE_DIR" -type f -name "*plant*.csv" | grep '_dstt_nwtt' | while read input_file; do
#find "$INPUT_BASE_DIR" -type f \( -name "*plant1*.csv" -o -name "*plant2*.csv" \) | grep 'TwoRobots' | while read -r input_file; do
   
    #input_rel_path="${input_file#$INPUT_BASE_DIR/}" 

    output_file="$OUTPUT_BASE_DIR/lqr-angle-dn-${counter}.csv"

    $PROGRAM -i "$input_file" -o "$output_file" -n 3
    ((counter++))
done

end=$(date +%s)
elapsed=$((end - start))

echo "Processed in $elapsed seconds."