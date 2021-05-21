import re

format = re.compile('\d+\,[AEIOU]\,-?(\d+)\,-?(\d+)\,-?(\d+)')
log_file_path = "data/dataFromNucleo.txt" #"dataset/log_file.txt"
dataset_path = "data/dataset.txt" #"dataset/dataset.txt"
with open(log_file_path,'r') as data_file:
    with open(dataset_path,'w') as out_file:
        for counter,line in enumerate(data_file):
            if re.search(format,line):
                out_file.write(line)

