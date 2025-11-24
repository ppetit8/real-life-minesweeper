#!/bin/bash


python3 train.py \
    --val_file /home/GTL/fjourda/ml4r/ws6_data/labels.txt \
    --train_file /home/GTL/fjourda/ml4r/ws6_data/labels.txt \
    --train_root /home/GTL/fjourda/ml4r/ws6_data \
    --val_root /home/GTL/fjourda/ml4r/ws6_data \
    --learning_rate 0.00001 \
    --batch_size 32 \
    --iter 1000 \
    --dropout 0.5 \
    --classes 3 \
    --output /home/GTL/fjourda/ml4r/ws6_data/output2

