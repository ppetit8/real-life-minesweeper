#!/bin/bash


python3 train.py \
    --val_file /home/GTL/fjourda/ml4r/ws6_data_reg/labels.txt \
    --train_file /home/GTL/fjourda/ml4r/ws6_data_reg/labels.txt \
    --train_root /home/GTL/fjourda/ml4r/ws6_data_reg \
    --val_root /home/GTL/fjourda/ml4r/ws6_data_reg \
    --learning_rate 0.00001 \
    --batch_size 32 \
    --iter 1000 \
    --dropout 0.5 \
    --classes 1 \
    --output /home/GTL/fjourda/ml4r/ws6_data_reg/output2

