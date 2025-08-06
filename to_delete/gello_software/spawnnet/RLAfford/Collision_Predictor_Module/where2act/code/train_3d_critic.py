"""
    Train the Action Scoring Module only
"""

import os
import time
import sys
import shutil
import random
from time import strftime
from argparse import ArgumentParser
import numpy as np
import torch
import torch.utils.data
import torch.nn.functional as F
from PIL import Image
from subprocess import call
from datagen import DataGen
from data import SAPIENVisionDataset
import Collision_Predictor_Module.where2act.code.w2a_utils as w2a_utils
from pointnet2_ops.pointnet2_utils import furthest_point_sample
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(BASE_DIR, 'blender_utils'))
import render_using_blender as render_utils


def train(conf, train_shape_list, train_data_list, val_data_list, all_train_data_list):
    # create training and validation datasets and data loaders
    data_features = ['pcs', 'pc_pxids', 'pc_movables', 'gripper_img_target', 'gripper_direction_camera', 'gripper_forward_direction_camera', \
            'result', 'cur_dir', 'shape_id', 'trial_id', 'is_original']
     
    # load network model
    model_def = w2a_utils.get_model_module(conf.model_version)

    # create models
    network = model_def.Network(conf.feat_dim)
    w2a_utils.printout(conf.flog, '\n' + str(network) + '\n')

    # create optimizers
    network_opt = torch.optim.Adam(network.parameters(), lr=conf.lr, weight_decay=conf.weight_decay)

    # learning rate scheduler
    network_lr_scheduler = torch.optim.lr_scheduler.StepLR(network_opt, step_size=conf.lr_decay_every, gamma=conf.lr_decay_by)

    # create logs
    if not conf.no_console_log:
        header = '     Time    Epoch     Dataset    Iteration    Progress(%)       LR    TotalLoss'
    if not conf.no_tb_log:
        # https://github.com/lanpa/tensorboard-pytorch
        from tensorboardX import SummaryWriter
        train_writer = SummaryWriter(os.path.join(conf.exp_dir, 'train'))
        val_writer = SummaryWriter(os.path.join(conf.exp_dir, 'val'))

    # send parameters to device
    network.to(conf.device)
    w2a_utils.optimizer_to_device(network_opt, conf.device)

    # load dataset
    train_dataset = SAPIENVisionDataset([conf.primact_type], conf.category_types, data_features, conf.buffer_max_num, \
            abs_thres=conf.abs_thres, rel_thres=conf.rel_thres, dp_thres=conf.dp_thres, img_size=conf.img_size, no_true_false_equal=conf.no_true_false_equal)
    
    val_dataset = SAPIENVisionDataset([conf.primact_type], conf.category_types, data_features, conf.buffer_max_num, \
            abs_thres=conf.abs_thres, rel_thres=conf.rel_thres, dp_thres=conf.dp_thres, img_size=conf.img_size, no_true_false_equal=conf.no_true_false_equal)
    val_dataset.load_data(val_data_list)
    w2a_utils.printout(conf.flog, str(val_dataset))
    
    val_dataloader = torch.utils.data.DataLoader(val_dataset, batch_size=conf.batch_size, shuffle=False, pin_memory=True, \
            num_workers=0, drop_last=True, collate_fn=w2a_utils.collate_feats, worker_init_fn=w2a_utils.worker_init_fn)
    val_num_batch = len(val_dataloader)

    # create a data generator
    datagen = DataGen(conf.num_processes_for_datagen, conf.flog)

    # sample succ
    if conf.sample_succ:
        sample_succ_list = []
        sample_succ_dirs = []

    # start training
    start_time = time.time()

    last_train_console_log_step, last_val_console_log_step = None, None

    # if resume
    start_epoch = 0
    if conf.resume:
        # figure out the latest epoch to resume
        for item in os.listdir(os.path.join(conf.exp_dir, 'ckpts')):
            if item.endswith('-train_dataset.pth'):
                start_epoch = max(start_epoch, int(item.split('-')[0]))

        # load states for network, optimizer, lr_scheduler, sample_succ_list
        data_to_restore = torch.load(os.path.join(conf.exp_dir, 'ckpts', '%d-network.pth' % start_epoch))
        network.load_state_dict(data_to_restore)
        data_to_restore = torch.load(os.path.join(conf.exp_dir, 'ckpts', '%d-optimizer.pth' % start_epoch))
        network_opt.load_state_dict(data_to_restore)
        data_to_restore = torch.load(os.path.join(conf.exp_dir, 'ckpts', '%d-lr_scheduler.pth' % start_epoch))
        network_lr_scheduler.load_state_dict(data_to_restore)

        # rmdir and make a new dir for the current sample-succ directory
        old_sample_succ_dir = os.path.join(conf.data_dir, 'epoch-%04d_sample-succ' % (start_epoch - 1))
        w2a_utils.force_mkdir(old_sample_succ_dir)

    # train for every epoch
    for epoch in range(start_epoch, conf.epochs):
        ### collect data for the current epoch
        if epoch > start_epoch:
            w2a_utils.printout(conf.flog, f'  [{strftime("%H:%M:%S", time.gmtime(time.time()-start_time)):>9s} Waiting epoch-{epoch} data ]')
            train_data_list = datagen.join_all()
            w2a_utils.printout(conf.flog, f'  [{strftime("%H:%M:%S", time.gmtime(time.time()-start_time)):>9s} Gathered epoch-{epoch} data ]')
            cur_data_folders = []
            for item in train_data_list:
                item = '/'.join(item.split('/')[:-1])
                if item not in cur_data_folders:
                    cur_data_folders.append(item)
            for cur_data_folder in cur_data_folders:
                with open(os.path.join(cur_data_folder, 'data_tuple_list.txt'), 'w') as fout:
                    for item in train_data_list:
                        if cur_data_folder == '/'.join(item.split('/')[:-1]):
                            fout.write(item.split('/')[-1]+'\n')

            # load offline-generated sample-random data
            for item in all_train_data_list:
                valid_id_l = conf.num_interaction_data_offline + conf.num_interaction_data * (epoch-1)
                valid_id_r = conf.num_interaction_data_offline + conf.num_interaction_data * epoch
                if valid_id_l <= int(item.split('_')[-1]) < valid_id_r:
                    train_data_list.append(item)

        ### start generating data for the next epoch
        # sample succ
        if conf.sample_succ:
            if conf.resume and epoch == start_epoch:
                sample_succ_list = torch.load(os.path.join(conf.exp_dir, 'ckpts', '%d-sample_succ_list.pth' % start_epoch))
            else:
                torch.save(sample_succ_list, os.path.join(conf.exp_dir, 'ckpts', '%d-sample_succ_list.pth' % epoch))
            for item in sample_succ_list:
                datagen.add_one_recollect_job(item[0], item[1], item[2], item[3], item[4], item[5], item[6])
            sample_succ_list = []
            sample_succ_dirs = []
            cur_sample_succ_dir = os.path.join(conf.data_dir, 'epoch-%04d_sample-succ' % epoch)
            w2a_utils.force_mkdir(cur_sample_succ_dir)

        # start all jobs
        datagen.start_all()
        w2a_utils.printout(conf.flog, f'  [ {strftime("%H:%M:%S", time.gmtime(time.time()-start_time)):>9s} Started generating epoch-{epoch+1} data ]')

        ### load data for the current epoch
        if conf.resume and epoch == start_epoch:
            train_dataset = torch.load(os.path.join(conf.exp_dir, 'ckpts', '%d-train_dataset.pth' % start_epoch))
        else:
            train_dataset.load_data(train_data_list)
        w2a_utils.printout(conf.flog, str(train_dataset))
        train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=conf.batch_size, shuffle=True, pin_memory=True, \
                num_workers=0, drop_last=True, collate_fn=w2a_utils.collate_feats, worker_init_fn=w2a_utils.worker_init_fn)
        train_num_batch = len(train_dataloader)

        ### print log
        if not conf.no_console_log:
            w2a_utils.printout(conf.flog, f'training run {conf.exp_name}')
            w2a_utils.printout(conf.flog, header)

        train_batches = enumerate(train_dataloader, 0)
        val_batches = enumerate(val_dataloader, 0)

        train_fraction_done = 0.0
        val_fraction_done = 0.0
        val_batch_ind = -1

        ### train for every batch
        for train_batch_ind, batch in train_batches:
            train_fraction_done = (train_batch_ind + 1) / train_num_batch
            train_step = epoch * train_num_batch + train_batch_ind

            log_console = not conf.no_console_log and (last_train_console_log_step is None or \
                    train_step - last_train_console_log_step >= conf.console_log_interval)
            if log_console:
                last_train_console_log_step = train_step
            
            # save checkpoint
            if train_batch_ind == 0:
                with torch.no_grad():
                    w2a_utils.printout(conf.flog, 'Saving checkpoint ...... ')
                    torch.save(network.state_dict(), os.path.join(conf.exp_dir, 'ckpts', '%d-network.pth' % epoch))
                    torch.save(network_opt.state_dict(), os.path.join(conf.exp_dir, 'ckpts', '%d-optimizer.pth' % epoch))
                    torch.save(network_lr_scheduler.state_dict(), os.path.join(conf.exp_dir, 'ckpts', '%d-lr_scheduler.pth' % epoch))
                    torch.save(train_dataset, os.path.join(conf.exp_dir, 'ckpts', '%d-train_dataset.pth' % epoch))
                    w2a_utils.printout(conf.flog, 'DONE')

            # set models to training mode
            network.train()

            # forward pass (including logging)
            total_loss, whole_feats, whole_pcs, whole_pxids, whole_movables = forward(batch=batch, data_features=data_features, network=network, conf=conf, is_val=False, \
                    step=train_step, epoch=epoch, batch_ind=train_batch_ind, num_batch=train_num_batch, start_time=start_time, \
                    log_console=log_console, log_tb=not conf.no_tb_log, tb_writer=train_writer, lr=network_opt.param_groups[0]['lr'])

            # optimize one step
            network_opt.zero_grad()
            total_loss.backward()
            network_opt.step()
            network_lr_scheduler.step()

            # sample succ
            if conf.sample_succ:
                network.eval()

                with torch.no_grad():
                    # sample a random EE orientation
                    random_up = torch.randn(conf.batch_size, 3).float().to(conf.device)
                    random_forward = torch.randn(conf.batch_size, 3).float().to(conf.device)
                    random_left = torch.cross(random_up, random_forward)
                    random_forward = torch.cross(random_left, random_up)
                    random_dirs1 = F.normalize(random_up, dim=1).float()
                    random_dirs2 = F.normalize(random_forward, dim=1).float()

                    # test over the entire image
                    whole_pc_scores1 = network.inference_whole_pc(whole_feats, random_dirs1, random_dirs2)     # B x N
                    whole_pc_scores2 = network.inference_whole_pc(whole_feats, -random_dirs1, random_dirs2)     # B x N

                    # add to the sample_succ_list if wanted
                    ss_cur_dir = batch[data_features.index('cur_dir')]
                    ss_shape_id = batch[data_features.index('shape_id')]
                    ss_trial_id = batch[data_features.index('trial_id')]
                    ss_is_original = batch[data_features.index('is_original')]
                    for i in range(conf.batch_size):
                        valid_id_l = conf.num_interaction_data_offline + conf.num_interaction_data * (epoch-1)
                        valid_id_r = conf.num_interaction_data_offline + conf.num_interaction_data * epoch

                        if ('sample-succ' not in ss_cur_dir[i]) and (ss_is_original[i]) and (ss_cur_dir[i] not in sample_succ_dirs) \
                                and (valid_id_l <= int(ss_trial_id[i]) < valid_id_r):
                            sample_succ_dirs.append(ss_cur_dir[i])

                            # choose one from the two options
                            gt_movable = whole_movables[i].cpu().numpy()

                            whole_pc_score1 = whole_pc_scores1[i].cpu().numpy() * gt_movable
                            whole_pc_score1[whole_pc_score1 < 0.5] = 0
                            whole_pc_score_sum1 = np.sum(whole_pc_score1) + 1e-12
                            
                            whole_pc_score2 = whole_pc_scores2[i].cpu().numpy() * gt_movable
                            whole_pc_score2[whole_pc_score2 < 0.5] = 0
                            whole_pc_score_sum2 = np.sum(whole_pc_score2) + 1e-12
                            
                            choose1or2_ratio = whole_pc_score_sum1 / (whole_pc_score_sum1 + whole_pc_score_sum2)
                            random_dir1 = random_dirs1[i].cpu().numpy()
                            random_dir2 = random_dirs2[i].cpu().numpy()
                            if np.random.random() < choose1or2_ratio:
                                whole_pc_score = whole_pc_score1
                            else:
                                whole_pc_score = whole_pc_score2
                                random_dir1 = -random_dir1

                            # sample <X, Y> on each img
                            pp = whole_pc_score + 1e-12
                            ptid = np.random.choice(len(whole_pc_score), 1, p=pp/pp.sum())
                            X = whole_pxids[i, ptid, 0].item()
                            Y = whole_pxids[i, ptid, 1].item()

                            # add job to the queue
                            str_cur_dir1 = ',' + ','.join(['%f' % elem for elem in random_dir1])
                            str_cur_dir2 = ',' + ','.join(['%f' % elem for elem in random_dir2])
                            sample_succ_list.append((conf.offline_data_dir, str_cur_dir1, str_cur_dir2, \
                                    ss_cur_dir[i].split('/')[-1], cur_sample_succ_dir, X, Y))

            # validate one batch
            while val_fraction_done <= train_fraction_done and val_batch_ind+1 < val_num_batch:
                val_batch_ind, val_batch = next(val_batches)

                val_fraction_done = (val_batch_ind + 1) / val_num_batch
                val_step = (epoch + val_fraction_done) * train_num_batch - 1

                log_console = not conf.no_console_log and (last_val_console_log_step is None or \
                        val_step - last_val_console_log_step >= conf.console_log_interval)
                if log_console:
                    last_val_console_log_step = val_step

                # set models to evaluation mode
                network.eval()

                with torch.no_grad():
                    # forward pass (including logging)
                    __ = forward(batch=val_batch, data_features=data_features, network=network, conf=conf, is_val=True, \
                            step=val_step, epoch=epoch, batch_ind=val_batch_ind, num_batch=val_num_batch, start_time=start_time, \
                            log_console=log_console, log_tb=not conf.no_tb_log, tb_writer=val_writer, lr=network_opt.param_groups[0]['lr'])
           

def forward(batch, data_features, network, conf, \
        is_val=False, step=None, epoch=None, batch_ind=0, num_batch=1, start_time=0, \
        log_console=False, log_tb=False, tb_writer=None, lr=None):
    # prepare input
    input_pcs = torch.cat(batch[data_features.index('pcs')], dim=0).to(conf.device)     # B x 3N x 3
    input_pxids = torch.cat(batch[data_features.index('pc_pxids')], dim=0).to(conf.device)     # B x 3N x 2
    input_movables = torch.cat(batch[data_features.index('pc_movables')], dim=0).to(conf.device)     # B x 3N
    batch_size = input_pcs.shape[0]

    input_pcid1 = torch.arange(batch_size).unsqueeze(1).repeat(1, conf.num_point_per_shape).long().reshape(-1)  # BN
    input_pcid2 = furthest_point_sample(input_pcs, conf.num_point_per_shape).long().reshape(-1)                 # BN
    input_pcs = input_pcs[input_pcid1, input_pcid2, :].reshape(batch_size, conf.num_point_per_shape, -1)
    input_pxids = input_pxids[input_pcid1, input_pcid2, :].reshape(batch_size, conf.num_point_per_shape, -1)
    input_movables = input_movables[input_pcid1, input_pcid2].reshape(batch_size, conf.num_point_per_shape)

    input_dirs1 = torch.cat(batch[data_features.index('gripper_direction_camera')], dim=0).to(conf.device)     # B x 3
    input_dirs2 = torch.cat(batch[data_features.index('gripper_forward_direction_camera')], dim=0).to(conf.device)     # B x 3

    # forward through the network
    pred_result_logits, pred_whole_feats = network(input_pcs, input_dirs1, input_dirs2)     # B x 2, B x F x N

    # prepare gt
    gt_result = torch.Tensor(batch[data_features.index('result')]).long().to(conf.device)     # B
    gripper_img_target = torch.cat(batch[data_features.index('gripper_img_target')], dim=0).to(conf.device)     # B x 3 x H x W
 
    # for each type of loss, compute losses per data
    result_loss_per_data = network.critic.get_ce_loss(pred_result_logits, gt_result)
    
    # for each type of loss, compute avg loss per batch
    result_loss = result_loss_per_data.mean()

    # compute total loss
    total_loss = result_loss

    # display information
    data_split = 'train'
    if is_val:
        data_split = 'val'

    with torch.no_grad():
        # log to console
        if log_console:
            w2a_utils.printout(conf.flog, \
                f'''{strftime("%H:%M:%S", time.gmtime(time.time()-start_time)):>9s} '''
                f'''{epoch:>5.0f}/{conf.epochs:<5.0f} '''
                f'''{data_split:^10s} '''
                f'''{batch_ind:>5.0f}/{num_batch:<5.0f} '''
                f'''{100. * (1+batch_ind+num_batch*epoch) / (num_batch*conf.epochs):>9.1f}%      '''
                f'''{lr:>5.2E} '''
                f'''{total_loss.item():>10.5f}''')
            conf.flog.flush()

        # log to tensorboard
        if log_tb and tb_writer is not None:
            tb_writer.add_scalar('total_loss', total_loss.item(), step)
            tb_writer.add_scalar('lr', lr, step)

        # gen visu
        if is_val and (not conf.no_visu) and epoch % conf.num_epoch_every_visu == 0:
            visu_dir = os.path.join(conf.exp_dir, 'val_visu')
            out_dir = os.path.join(visu_dir, 'epoch-%04d' % epoch)
            input_pc_dir = os.path.join(out_dir, 'input_pc')
            gripper_img_target_dir = os.path.join(out_dir, 'gripper_img_target')
            info_dir = os.path.join(out_dir, 'info')
            
            if batch_ind == 0:
                # create folders
                os.mkdir(out_dir)
                os.mkdir(input_pc_dir)
                os.mkdir(gripper_img_target_dir)
                os.mkdir(info_dir)

            if batch_ind < conf.num_batch_every_visu:
                w2a_utils.printout(conf.flog, 'Visualizing ...')
                for i in range(batch_size):
                    fn = 'data-%03d.png' % (batch_ind * batch_size + i)
                    render_utils.render_pts(os.path.join(BASE_DIR, input_pc_dir, fn), input_pcs[i].cpu().numpy(), highlight_id=0)
                    cur_gripper_img_target = (gripper_img_target[i].permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
                    Image.fromarray(cur_gripper_img_target).save(os.path.join(gripper_img_target_dir, fn))
                    with open(os.path.join(info_dir, fn.replace('.png', '.txt')), 'w') as fout:
                        fout.write('cur_dir: %s\n' % batch[data_features.index('cur_dir')][i])
                        fout.write('pred: %s\n' % w2a_utils.print_true_false((pred_result_logits[i]>0).cpu().numpy()))
                        fout.write('gt: %s\n' % w2a_utils.print_true_false(gt_result[i].cpu().numpy()))
                        fout.write('result_loss: %f\n' % result_loss_per_data[i].item())
                
            if batch_ind == conf.num_batch_every_visu - 1:
                # visu html
                w2a_utils.printout(conf.flog, 'Generating html visualization ...')
                sublist = 'input_pc,gripper_img_target,info'
                cmd = 'cd %s && python %s . 10 htmls %s %s > /dev/null' % (out_dir, os.path.join(BASE_DIR, 'gen_html_hierachy_local.py'), sublist, sublist)
                call(cmd, shell=True)
                w2a_utils.printout(conf.flog, 'DONE')

    return total_loss, pred_whole_feats.detach(), input_pcs.detach(), input_pxids.detach(), input_movables.detach()


if __name__ == '__main__':
    ### get parameters
    parser = ArgumentParser()
    
    # main parameters (required)
    parser.add_argument('--exp_suffix', type=str, help='exp suffix')
    parser.add_argument('--model_version', type=str, help='model def file')
    parser.add_argument('--primact_type', type=str, help='the primact type')
    parser.add_argument('--category_types', type=str, help='list all categories [Default: None, meaning all 10 categories]', default=None)
    parser.add_argument('--data_dir_prefix', type=str, help='data directory')
    parser.add_argument('--offline_data_dir', type=str, help='data directory')
    parser.add_argument('--val_data_dir', type=str, help='data directory')
    parser.add_argument('--val_data_fn', type=str, help='data directory', default='data_tuple_list_val_subset.txt')
    parser.add_argument('--train_shape_fn', type=str, help='training shape file that indexs all shape-ids')
    parser.add_argument('--ins_cnt_fn', type=str, help='a file listing all category instance count')

    # main parameters (optional)
    parser.add_argument('--device', type=str, default='cuda:0', help='cpu or cuda:x for using cuda on GPU number x')
    #parser.add_argument('--seed', type=int, default=3124256514, help='random seed (for reproducibility) [specify -1 means to generate a random one]')
    parser.add_argument('--seed', type=int, default=-1, help='random seed (for reproducibility) [specify -1 means to generate a random one]')
    parser.add_argument('--log_dir', type=str, default='logs', help='exp logs directory')
    parser.add_argument('--overwrite', action='store_true', default=False, help='overwrite if exp_dir exists [default: False]')
    parser.add_argument('--resume', action='store_true', default=False, help='resume if exp_dir exists [default: False]')

    # network settings
    parser.add_argument('--img_size', type=int, default=224)
    parser.add_argument('--num_point_per_shape', type=int, default=10000)
    parser.add_argument('--feat_dim', type=int, default=128)
    parser.add_argument('--abs_thres', type=float, default=0.01, help='abs thres')
    parser.add_argument('--rel_thres', type=float, default=0.5, help='rel thres')
    parser.add_argument('--dp_thres', type=float, default=0.5, help='dp thres')
    parser.add_argument('--no_true_false_equal', action='store_true', default=False, help='if make the true/false data loaded equally [default: False]')

    # training parameters
    parser.add_argument('--epochs', type=int, default=10000)
    parser.add_argument('--buffer_max_num', type=int, default=20000)
    parser.add_argument('--num_processes_for_datagen', type=int, default=20)
    parser.add_argument('--num_interaction_data_offline', type=int, default=5)
    parser.add_argument('--num_interaction_data', type=int, default=1)
    parser.add_argument('--batch_size', type=int, default=32)
    parser.add_argument('--lr', type=float, default=0.001)
    parser.add_argument('--weight_decay', type=float, default=1e-5)
    parser.add_argument('--lr_decay_by', type=float, default=0.9)
    parser.add_argument('--lr_decay_every', type=float, default=5000)
    parser.add_argument('--sample_succ', action='store_true', default=False)

    # loss weights

    # logging
    parser.add_argument('--no_tb_log', action='store_true', default=False)
    parser.add_argument('--no_console_log', action='store_true', default=False)
    parser.add_argument('--console_log_interval', type=int, default=10, help='number of optimization steps beween console log prints')

    # visu
    parser.add_argument('--num_batch_every_visu', type=int, default=1, help='num batch every visu')
    parser.add_argument('--num_epoch_every_visu', type=int, default=10, help='num epoch every visu')
    parser.add_argument('--no_visu', action='store_true', default=False, help='no visu? [default: False]')

    # parse args
    conf = parser.parse_args()


    ### prepare before training
    # make exp_name
    conf.exp_name = f'exp-{conf.model_version}-{conf.primact_type}-{conf.category_types}-{conf.exp_suffix}'

    if conf.overwrite and conf.resume:
        raise ValueError('ERROR: cannot specify both --overwrite and --resume!')

    # mkdir exp_dir; ask for overwrite if necessary; or resume
    conf.exp_dir = os.path.join(conf.log_dir, conf.exp_name)
    if os.path.exists(conf.exp_dir):
        if not conf.resume:
            if not conf.overwrite:
                response = input('A training run named "%s" already exists, overwrite? (y/n) ' % conf.exp_name)
                if response != 'y':
                    exit(1)
            shutil.rmtree(conf.exp_dir)
    else:
        if conf.resume:
            raise ValueError('ERROR: no training run named %s to resume!' % conf.exp_name)
    if not conf.resume:
        os.mkdir(conf.exp_dir)
        os.mkdir(os.path.join(conf.exp_dir, 'ckpts'))
        if not conf.no_visu:
            os.mkdir(os.path.join(conf.exp_dir, 'val_visu'))

    # prepare data_dir
    conf.data_dir = conf.data_dir_prefix + '-' + conf.exp_name
    if os.path.exists(conf.data_dir):
        if not conf.resume:
            if not conf.overwrite:
                response = input('A data_dir named "%s" already exists, overwrite? (y/n) ' % conf.data_dir)
                if response != 'y':
                    exit(1)
            shutil.rmtree(conf.data_dir)
    else:
        if conf.resume:
            raise ValueError('ERROR: no data_dir named %s to resume!' % conf.data_dir)
    if not conf.resume:
        os.mkdir(conf.data_dir)

    # control randomness
    if conf.seed < 0:
        conf.seed = random.randint(1, 10000)
    random.seed(conf.seed)
    np.random.seed(conf.seed)
    torch.manual_seed(conf.seed)

    # save config
    if not conf.resume:
        torch.save(conf, os.path.join(conf.exp_dir, 'conf.pth'))

    # file log
    if conf.resume:
        flog = open(os.path.join(conf.exp_dir, 'train_log.txt'), 'a+')
    else:
        flog = open(os.path.join(conf.exp_dir, 'train_log.txt'), 'w')
    conf.flog = flog

    # backup command running
    w2a_utils.printout(flog, ' '.join(sys.argv) + '\n')
    w2a_utils.printout(flog, f'Random Seed: {conf.seed}')

    # backup python files used for this training
    if not conf.resume:
        os.system('cp datagen.py data.py models/%s.py %s %s' % (conf.model_version, __file__, conf.exp_dir))
     
    # set training device
    device = torch.device(conf.device)
    w2a_utils.printout(flog, f'Using device: {conf.device}\n')
    conf.device = device
    
    # parse params
    w2a_utils.printout(flog, 'primact_type: %s' % str(conf.primact_type))

    if conf.category_types is None:
        conf.category_types = ['Box', 'Door', 'Faucet', 'Kettle', 'Microwave', 'Refrigerator', 'StorageFurniture', 'Switch', 'TrashCan', 'Window']
    else:
        conf.category_types = conf.category_types.split(',')
    w2a_utils.printout(flog, 'category_types: %s' % str(conf.category_types))
    
    # read cat2freq
    conf.cat2freq = dict()
    with open(conf.ins_cnt_fn, 'r') as fin:
        for l in fin.readlines():
            category, _, freq = l.rstrip().split()
            conf.cat2freq[category] = int(freq)
    w2a_utils.printout(flog, str(conf.cat2freq))

    # read train_shape_fn
    train_shape_list = []
    with open(conf.train_shape_fn, 'r') as fin:
        for l in fin.readlines():
            shape_id, category = l.rstrip().split()
            if category in conf.category_types:
                train_shape_list.append((shape_id, category))
    w2a_utils.printout(flog, 'len(train_shape_list): %d' % len(train_shape_list))
    
    with open(os.path.join(conf.offline_data_dir, 'data_tuple_list.txt'), 'r') as fin:
        all_train_data_list = [os.path.join(conf.offline_data_dir, l.rstrip()) for l in fin.readlines()]
    w2a_utils.printout(flog, 'len(all_train_data_list): %d' % len(all_train_data_list))
    if conf.resume:
        train_data_list = None
    else:
        train_data_list = []
        for item in all_train_data_list:
            if int(item.split('_')[-1]) < conf.num_interaction_data_offline:
                train_data_list.append(item)
        w2a_utils.printout(flog, 'len(train_data_list): %d' % len(train_data_list))
    
    with open(os.path.join(conf.val_data_dir, conf.val_data_fn), 'r') as fin:
        val_data_list = [os.path.join(conf.val_data_dir, l.rstrip()) for l in fin.readlines()]
    w2a_utils.printout(flog, 'len(val_data_list): %d' % len(val_data_list))
     
    ### start training
    train(conf, train_shape_list, train_data_list, val_data_list, all_train_data_list)


    ### before quit
    # close file log
    flog.close()

