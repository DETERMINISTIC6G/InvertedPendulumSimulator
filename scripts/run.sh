#!/bin/bash

INPUT_BASE_DIR="truncnormal_5_5"
OUTPUT_BASE_DIR="results/truncnormal_5_5/pid-angle-dn"
PROGRAM="./simulate-event_queue"


mkdir -p "$OUTPUT_BASE_DIR"
pdc_ND_10_5
counter=0
start=$(date +%s)

find "$INPUT_BASE_DIR" -type f -name "*plant*.csv" | grep '_dstt_nwtt' | while read input_file; do
   
    #input_rel_path="${input_file#$INPUT_BASE_DIR/}" 

    output_file="$OUTPUT_BASE_DIR/pid-angle-dn-${counter}.csv"

    $PROGRAM -i "$input_file" -o "$output_file" -n 1
    ((counter++))
done

end=$(date +%s)
elapsed=$((end - start))

echo "Processed in $elapsed seconds."