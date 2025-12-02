#!/bin/bash


python3 train_regression.py \
    --val_file /home/GTL/fjourda/ml4r/ws6_data_reg_test/labels.txt \
    --train_file /home/GTL/fjourda/ml4r/ws6_data_reg/labels.txt \
    --train_root /home/GTL/fjourda/ml4r/ws6_data_reg \
    --val_root /home/GTL/fjourda/ml4r/ws6_data_reg_test \
    --learning_rate 0.00001 \
    --batch_size 32 \
    --iter 5000 \
    --dropout 0.5 \
    --classes 1 \
    --output /home/GTL/fjourda/ml4r/ws6_data_reg/output2

