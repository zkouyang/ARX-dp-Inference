# %%
import time
import numpy as np
import click
import cv2
import numpy as np
import torch
import dill
import hydra
import pathlib
import skvideo.io
from omegaconf import OmegaConf
from einops import rearrange
import scipy.spatial.transform as st
# from diffusion_policy.real_world.real_env import RealEnv
# from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse
from diffusion_policy.common.precise_sleep import precise_wait
from diffusion_policy.real_world.real_inference_util import (
    get_real_obs_resolution, 
    get_real_obs_dict)
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.cv2_util import get_image_transform

import ipdb

OmegaConf.register_new_resolver("eval", eval, replace=True)

## data configuration settings
@click.command()
@click.option('--input', '-i', required=True, help='Path to checkpoint')
@click.option('--output', '-o', required=True, help='Directory to save recording')
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
@click.option('--steps_per_inference', '-si', default=6, type=int, help="Action horizon for inference.")
@click.option('--max_timesteps', '-md', default=500, help='Max duration for each epoch in seconds.')
@click.option('--frequency', '-f', default=50, type=float, help="Control frequency in Hz.")
@click.option('--num_inference_steps', '-n', default=16, type=int, help="DDIM inference iterations.")
def main(input, output,
    vis_camera_idx, 
    steps_per_inference, 
    max_timesteps,
    frequency,
    num_inference_steps):

    ### load checkpoint
    ckpt_path = input
    payload = torch.load(open(ckpt_path, 'rb'), pickle_module=dill) ## payload = {'cfg', 'state_dicts', 'pickles'}
    cfg = payload['cfg']
    cls = hydra.utils.get_class(cfg._target_)
    workspace = cls(cfg)
    workspace: BaseWorkspace
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    ### load policy
    action_offset = 0
    delta_action = False
    if 'diffusion' in cfg.name:
        # diffusion model
        policy: BaseImagePolicy
        policy = workspace.model
        if cfg.training.use_ema:
            policy = workspace.ema_model

        device = torch.device('cuda')
        policy.eval().to(device)

        # set inference params
        policy.num_inference_steps = num_inference_steps #16 # [DDIM inference iterations]
        #policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1
    else:
        raise RuntimeError("Unsupported policy type: ", cfg.name)
    
    ### hyper-parameters
    state_dim = cfg.task.shape_meta.obs.qpos.shape[0] ## qpos shape
    camera_names = cfg.task.camera_names
    shape_meta = cfg.task.shape_meta
    c, h, w = shape_meta.obs.images.shape ## [c, h, w]

    ### setup experiment
    dt = 1/frequency ## Warning: ALOHA constants.py 中的 DT = 0.02, so set frequency = 50

    obs_res = get_real_obs_resolution(cfg.task.shape_meta)
    n_obs_steps = cfg.n_obs_steps
    print("n_obs_steps: ", n_obs_steps)
    print("max_timesteps:", max_timesteps)
    print("action_offset:", action_offset) ## what is action offset?

    ## load aloha env
    from aloha.aloha_scripts.robot_utils import move_grippers
    from aloha.aloha_scripts.real_env import make_real_env

    env = make_real_env(init_node=True)
    env_max_reward = 0

    ## rollout
    max_timesteps = int(max_timesteps * 1) ## may increase for real-world tasks

    num_rollouts = 1
    episode_returns = []
    highest_rewards = []

    for rollout_idx in range(num_rollouts):
        rollout_idx += 1
        
        print(f"Rollout {rollout_idx} - Collecting observations...")

        ## reset env
        ts = env.reset() 
        t_idx = n_obs_steps

        qpos_history = np.zeros((1, max_timesteps+n_obs_steps, state_dim)) ## [1, max_timesteps, state_dim]
        images_history = np.zeros((1, max_timesteps+n_obs_steps, c, h, w)) ## [1, max_timesteps, c, h, w]

        qpos_history, images_history = collect_obs(ts, t_idx, n_obs_steps, qpos_history, images_history, camera_names, shape_meta)

        with torch.inference_mode():
            ## loop max_timesteps
            while True:
                ''' construct observations_seq = {"images", "qpos"} '''
                obs_dict_np = dict()
                obs_dict_np["images"] = images_history[0][t_idx-n_obs_steps:t_idx] ## [1, n_obs_steps, c, h, w]
                obs_dict_np["qpos"] = qpos_history[0][t_idx-n_obs_steps:t_idx] ## [1, n_obs_steps, state_dim]")

                print(f"observation_range = {t_idx-n_obs_steps}:{t_idx}")
                # ipdb.set_trace()

                ''' get action sequence '''
                s = time.time()
                obs_dict = dict_apply(obs_dict_np, 
                    lambda x: torch.from_numpy(x).unsqueeze(0).to(device))
                result = policy.predict_action(obs_dict)
                print(f"Execution Policy: {time.time() - s:.3f} seconds")

                action_seq = result['action'][0].detach().to('cpu').numpy()
                
                ''' implement action sequence '''
                for action in action_seq:
                    ts = env.step(action)

                    qpos_history, images_history = collect_obs(ts, t_idx, n_obs_steps, qpos_history, images_history, camera_names, shape_meta)

                    t_idx += 1
                    if t_idx == max_timesteps+n_obs_steps:
                        break
                
                if t_idx >= max_timesteps+n_obs_steps:
                    break

    ### move grippers
    PUPPET_GRIPPER_JOINT_OPEN = 1.4910
    move_grippers([env.puppet_bot_left, env.puppet_bot_right], [PUPPET_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)  # open
    pass

    #     ## statistics
    #     save_videos(image_list, DT, video_path=os.path.join(ckpt_dir, f'video{rollout_id}.mp4'))
                        

def collect_obs(ts, idx, n_obs_steps, qpos_history, images_history, camera_names, shape_meta):
    obs = ts.observation
    ## get qpos input
    qpos_numpy = np.array(obs['qpos'])
    qpos = np.expand_dims(qpos_numpy, axis=0) ## [1, state_dim]
    if idx == n_obs_steps:
        qpos_history[:, idx-n_obs_steps:idx] = qpos
    else:
        qpos_history[:, idx] = qpos

    ## get image input
    curr_image = get_image(ts, camera_names, shape_meta)
    if idx == n_obs_steps:
        images_history[:, idx-n_obs_steps:idx] = curr_image
    else:
        images_history[:, idx] = curr_image

    return qpos_history, images_history



def get_image(ts, camera_names, shape_meta):

    curr_images = []
    for cam_name in camera_names:
        curr_image = rearrange(ts.observation['images'][cam_name], 'h w c -> c h w') / 255.0
        curr_images.append(curr_image)

    ## align with aloha_datasets
    curr_image = np.concatenate(curr_images, axis=1) 
    assert curr_image.shape == tuple(shape_meta.obs.images.shape)
    curr_image = np.expand_dims(curr_image, axis=0)
     ## [T=1, c, h, w]
    return curr_image



def save_videos(video, dt, video_path=None):
    if isinstance(video, list):
        cam_names = list(video[0].keys())
        h, w, _ = video[0][cam_names[0]].shape
        w = w * len(cam_names)
        fps = int(1/dt)
        out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        for ts, image_dict in enumerate(video):
            images = []
            for cam_name in cam_names:
                image = image_dict[cam_name]
                image = image[:, :, [2, 1, 0]] # swap B and R channel
                images.append(image)
            images = np.concatenate(images, axis=1)
            out.write(images)
        out.release()
        print(f'Saved video to: {video_path}')
    elif isinstance(video, dict):
        cam_names = list(video.keys())
        all_cam_videos = []
        for cam_name in cam_names:
            all_cam_videos.append(video[cam_name])
        all_cam_videos = np.concatenate(all_cam_videos, axis=2) # width dimension

        n_frames, h, w, _ = all_cam_videos.shape
        fps = int(1 / dt)
        out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        for t in range(n_frames):
            image = all_cam_videos[t]
            image = image[:, :, [2, 1, 0]]  # swap B and R channel
            out.write(image)
        out.release()
        print(f'Saved video to: {video_path}')



if __name__ == '__main__':
    main()


