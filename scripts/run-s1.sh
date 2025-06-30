#!/bin/bash

INPUT_BASE_DIR="5g-1-pdc-cycle20ms"
OUTPUT_BASE_DIR="ref-test/5g-2a/lqr-angle-dn"
PROGRAM="./simulate-event_queue"


mkdir -p "$OUTPUT_BASE_DIR"

counter=0
start=$(date +%s)

find "$INPUT_BASE_DIR" -type f -name "*plant*.csv" | grep '_dstt_nwtt' | while read input_file; do
   
    output_file="$OUTPUT_BASE_DIR/lqr-angle-dn-${counter}.csv"

    $PROGRAM -i "$input_file" -o "$output_file" -n 2
    ((counter++))
done

end=$(date +%s)
elapsed=$((end - start))

echo "Processed in $elapsed seconds."