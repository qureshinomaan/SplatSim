import os
import sys
from argparse import ArgumentParser

from datagen import DataGen

parser = ArgumentParser()
parser.add_argument('--data_dir', type=str, help='data directory')
parser.add_argument('--data_fn', type=str, help='data file that indexs all shape-ids')
parser.add_argument('--primact_types', type=str, help='list all primacts [separated by comma], default: None, meaning all', default=None)
parser.add_argument('--category_types', type=str, help='list all categories [separated by comma], default: None, meaning all', default=None)
parser.add_argument('--num_processes', type=int, default=40, help='number of CPU cores to use')
parser.add_argument('--num_epochs', type=int, default=200, help='control the data amount')
parser.add_argument('--starting_epoch', type=int, default=0, help='if you want to run this data generation across multiple machines, you can set this parameter so that multiple data folders generated on different machines have continuous trial-id for easier merging of multiple datasets')
parser.add_argument('--out_fn', type=str, default=None, help='a file that lists all valid interaction data collection [default: None, meaning data_tuple_list.txt]. Again, this is used when you want to generate data across multiple machines. You can store the filelist on different files and merge them together to get one data_tuple_list.txt')
parser.add_argument('--ins_cnt_fn', type=str, help='a file listing all category instance count, which is used to balance the interaction data amount to make sure that all categories have roughly same amount of data interaction, regardless of different shape counts in these categories')
conf = parser.parse_args()
    
if conf.out_fn is None:
    conf.out_fn = 'data_tuple_list.txt'

if conf.primact_types is None:
    conf.primact_types = ['pushing', 'pushing-up', 'pushing-left', 'pulling', 'pulling-up', 'pulling-left']
else:
    conf.primact_types = conf.primact_types.split(',')
print(conf.primact_types)

if conf.category_types is None:
    conf.category_types = ['Box', 'Bucket', 'Door', 'Faucet', 'Kettle', 'KitchenPot', 'Microwave', 'Refrigerator', \
            'Safe', 'StorageFurniture', 'Switch', 'Table', 'TrashCan', 'WashingMachine', 'Window']
else:
    conf.category_types = conf.category_types.split(',')
print(conf.category_types)

cat2freq = dict()
with open(conf.ins_cnt_fn, 'r') as fin:
    for l in fin.readlines():
        cat, _, freq = l.rstrip().split()
        cat2freq[cat] = int(freq)
print(cat2freq)

datagen = DataGen(conf.num_processes)

with open(conf.data_fn, 'r') as fin:
    for l in fin.readlines():
        shape_id, cat = l.rstrip().split()
        if cat in conf.category_types:
            for primact_type in conf.primact_types:
                for epoch in range(conf.starting_epoch, conf.starting_epoch+conf.num_epochs):
                    for cnt_id in range(cat2freq[cat]):
                        #print(shape_id, cat, epoch, cnt_id)
                        datagen.add_one_collect_job(conf.data_dir, shape_id, cat, cnt_id, primact_type, epoch)

datagen.start_all()

data_tuple_list = datagen.join_all()
with open(os.path.join(conf.data_dir, conf.out_fn), 'w') as fout:
    for item in data_tuple_list:
        fout.write(item.split('/')[-1]+'\n')

