import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--train', type=int, default="-1")
parser.add_argument('--test', type=int, default="-1")
parser.add_argument('--multiplier', type=int, default="8")
parser.add_argument('--var_size', type=int, default="1")
args = parser.parse_args()

training_set_multiplier = args.multiplier
variance_set_multiplier = args.var_size

if args.train == -1 or args.test == -1 :
    train_success_list = [float(s) for s in input("train success rate: ").split(' ')]
    test_success_list = [float(s) for s in input("test success rate: ").split(' ')]
else :
    total_list = [float(s) for s in input("all success rate: ").split(' ')]
    assert(len(total_list) % (args.train + args.test) == 0)
    training_set_multiplier = len(total_list) // (args.train + args.test)
    print(training_set_multiplier)
    train_success_list = total_list[:args.train*training_set_multiplier]
    test_success_list = total_list[args.train*training_set_multiplier:]

def ASR(success_list) :                 # average success rate
    num_envs = len(success_list)
    num_objs = len(success_list)//training_set_multiplier
    arr = np.array(success_list)
    var_arr = arr.reshape(num_objs, training_set_multiplier//variance_set_multiplier, variance_set_multiplier)
    return np.mean(arr), np.var(np.mean(var_arr, axis=(1,2)))#np.mean(np.var(var_arr, axis=1))

def MR(success_list) :                  # master rate
    per_cabinet_array = np.array(success_list).reshape(len(success_list)//training_set_multiplier, training_set_multiplier//variance_set_multiplier, variance_set_multiplier)
    # print(per_cabinet_array)
    per_cabinet_mean = np.mean(per_cabinet_array, axis=2) >= 0.5
    # print(per_cabinet_mean)
    var_arr = np.var(per_cabinet_mean, axis=1)
    return np.mean(np.mean(per_cabinet_mean, axis=1)), np.mean(var_arr)

print(ASR(train_success_list), ASR(test_success_list), MR(train_success_list), MR(test_success_list))
# print(train_success_list, test_success_list)
