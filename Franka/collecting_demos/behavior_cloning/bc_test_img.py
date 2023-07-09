import numpy as np
import sys
import torch
import select
import tty
import termios
import torch.nn as nn
import torch.functional as F 
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader
from typing import Tuple
from robot.data import RoboDemoDset
from robot.franka_env import LowDimRobotEnv
from robot.utils import HZ

DEMO = "GOAL3"
PROG = 500
EXPERT = RoboDemoDset(f'/home/siri/Projects/rewardlearning-robot/data/demos/{DEMO}/demos.hdf', read_only_if_exists=False)
EXPERT._process_rgb_to_r3m_vecs()
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Behavior Cloning neural network
class BCNet(nn.Module):
    def __init__(self) -> None:
        super(BCNet, self).__init__()
        self.fc1 = nn.Linear(2048, 400)
        self.fc2 = nn.Linear(400, 800)
        self.fc3 = nn.Linear(800, 800)
        self.fc4 = nn.Linear(800, 400)
        self.fc5 = nn.Linear(400, 4)
        self.relu = nn.ReLU()
    
    def forward(self, x):
        x = self.fc1(x)
        x = self.relu(x)    
        x = self.fc2(x)
        x = self.relu(x)
        x = self.fc3(x)
        x = self.relu(x)
        x = self.fc4(x)
        x = self.relu(x)
        x = self.fc5(x)
        return x

# Extracting data correctly from datasets
class BCDataset(Dataset):
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def __len__(self):
        return(len(self.x))

    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]
    
def test_data(): #### TODO: look in the end
    expert_data = EXPERT 
    data = expert_data.__getitem__(13)
    deltas = data['eef_actions']
    # poses = data['eef_pose']
    env = LowDimRobotEnv(ip_address="127.0.0.1", hz=HZ)
    env.reset()

    for delta in deltas:
        env.step(delta)

# loading train and test datasets with scale     
def process_data(SCALE) -> Tuple[DataLoader, DataLoader]:
    expert_data = EXPERT
   
    # state = []
    image = []
    targets = []
    trajs = []
    for traj in expert_data:
        trajs.append(trajs)
        # ee_poses = traj['eef_pose']
        # deltas = traj['eef_delta']
        rgb_vecs = traj['r3m_vec']
        deltas = traj['eef_actions']

        for i in range(1, len(rgb_vecs)-1):
            image.append(rgb_vecs[i-1] / SCALE)
            targets.append(deltas[i])
        
    image, targets = torch.Tensor(np.array(image)), torch.Tensor(np.array(targets))

    perm = torch.randperm(len(image))
    image, targets = image[perm], targets[perm]

    test_idx = int(len(image) * 0.95) 
    train_x, train_y = image[:test_idx], targets[:test_idx]
    test_x, test_y = image[test_idx:], targets[test_idx:]

    train_loader = DataLoader(BCDataset(train_x, train_y), batch_size=32, shuffle=True)
    test_loader = DataLoader(BCDataset(test_x, test_y), batch_size=128, shuffle=True)

    return train_loader, test_loader 

def one_train_epoch(net, optimizer, train_loader):
    net.train()
    for batch in train_loader:
        obs, act = batch
        obs = obs.cuda()
        act = act.cuda()
        act_pred = net(obs)
        
        loss = torch.sum((act_pred - act)**2, dim=-1).mean(dim=0)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return loss.item()

def eval(net, test_loader):
    criterion = nn.MSELoss()
    with torch.no_grad():
        losses = []
        for data, labels in test_loader:
            data, labels = data.to(device), labels.to(device)
            preds = net(data)
            # print("preds: ", preds)
            # print("labels: ", labels)
            loss = criterion(preds, labels)
            loss = loss.item()
            losses.append(loss)
        # print(f'Average test loss is {np.mean(losses)}')
        return np.mean(losses)
    
def vis_traj(net, output_name, save=False):
    expert_data = EXPERT
    obs = expert_data[0]['eef_pose']
    act = expert_data[0]['eef_actions']
    obs_torch = torch.from_numpy(obs).cuda()
    out_ac = net(obs_torch)
    fig, ax = plt.subplots(4, 1)
    out_ac_np = out_ac.detach().cpu().numpy()
    for j in range(4):
        ax[j].plot(out_ac_np[:, j], color='red')
        ax[j].plot(act[:, j], color='blue')
    plt.savefig(f'{output_name}')
    if save:
        torch.save(out_ac, "out_ac.pt")
    plt.clf()
    plt.close("all")

def hyp_search():
    train_loader, test_loader = process_data(1)
    lrs = [0.01, 0.001, 0.0001, 0.00001]
    wds = [0, 0.001, 0.0001, 0.00001, 0.000001] 
    eps = [500, 800, 1000, 1500]
    best = 1000
    best_comb = [0, 0, 0]
    for lr in lrs:
        for wd in wds:
            for ep in eps:
                net = BCNet().to(device)
                optimizer = torch.optim.Adam(net.parameters(), lr=lr, weight_decay=wd) # best hyper: nonlinear = LReLU, lr = 0.00001, decay = 0.01
                epochs = ep
                train_losses = []
                valid_losses = []
                for _ in range(epochs): 
                    train_loss = one_train_epoch(net, optimizer, train_loader)
                    valid_loss = eval(net, test_loader)
                    train_losses.append(train_loss)
                    valid_losses.append(valid_loss)
                cur_loss = np.mean(valid_losses)
                if cur_loss < best:
                    best = cur_loss
                    best_comb[0] = lr
                    best_comb[1] = wd
                    best_comb[2] = ep
    print(f"\nBest: {best}")
    print(f"Best Combo: {best_comb}")

def try_until_estop(model, SCALE):
    env = LowDimRobotEnv(ip_address="127.0.0.1", hz=HZ)
    obs = env.reset()
    model.eval()
    model.double()
    with torch.no_grad():
        while(True):
            pos = torch.tensor(obs['lowdim_ee'] / SCALE).double().to(device)
            print(pos)
            update = np.array(model(pos).cpu())
            print(update * SCALE)
            obs = env.step(action=(update*SCALE), delta=True)[0]

def main():
    mode = sys.argv[1]
    scale = float(sys.argv[2])
    if mode == "train":
        epoch_num = int(sys.argv[3]) # 800 #550 gives good set of learning curves
        train_loader, test_loader = process_data(scale)
        net = BCNet().cuda()
        optimizer = torch.optim.Adam(net.parameters(), lr=0.01, weight_decay=1e-06)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=2000, eta_min=1e-6)
        # scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=100, gamma=0.95)
        train_losses = []
        valid_losses = []
        for j in range(epoch_num):
            train_loss = one_train_epoch(net, optimizer, train_loader)
            valid_loss = eval(net, test_loader)
            scheduler.step()

            print("Epoch %d loss is %f"%(j, train_loss))

            train_losses.append(train_loss)
            valid_losses.append(valid_loss)

        print("saving model...")
        torch.save(net, f"/home/siri/Projects/rewardlearning-robot/behavior_cloning/img.pt")
        print("saved, terminate.")

        print("plotting training data")
        plt.plot(train_losses, label='train')
        plt.plot(valid_losses, label='validation')
        plt.legend()
        plt.show()

    elif mode == "deploy":
        deploy(scale)
    elif mode == "test_data":
        process_data(1)
    elif mode == "search":
        hyp_search()
    elif mode == "predict":
        traj_num = int(sys.argv[3])
        pred_replay(traj_num)
    else: 
        print("Wrong argument, please enter either 'train' or 'deploy'")

def pred_replay(num):
    expert_data = EXPERT
    net = torch.load(f"/home/siri/Projects/rewardlearning-robot/behavior_cloning/{DEMO}.pt")
    obs = expert_data[num]['eef_pose']
    act = expert_data[num]['eef_actions']
    obs_torch = torch.from_numpy(obs).cuda()
    out_ac = net(obs_torch)
    fig, ax = plt.subplots(4, 1)
    out_ac_np = out_ac.detach().cpu().numpy()
    for j in range(4):
        ax[j].plot(out_ac_np[:, j], color='red')
        ax[j].plot(act[:, j], color='blue')
    plt.savefig(f'test_bc_pred{num}.png')
    env = LowDimRobotEnv(ip_address="127.0.0.1", hz=HZ)
    net.eval()
    net.double()
    while True:
            obs = env.reset()
            # preds = torch.load("out_ac.pt")
            print('press "a" then "enter" to start, hit "a" again to stop')
            console = input()
            a = console == 'a'
            d = console == 'd'
            while not a and not d:
                console = input()
                a = console == 'a'
                d = console == 'd'

            if d:
                break

            def isData():
                return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                with torch.no_grad():
                    while True:
                        # Get Button Input (only if True) --> handle extended button press...
                        # Terminate...
                        if isData():
                            c = sys.stdin.read(1)
                            print(c)
                            if c == 'a':  # x1b is ESC
                                print("\tHit (a) stop deploying\n")
                                break

                        for pred in out_ac: 
                            act = np.array(pred.cpu())
                            env.step(action=act)
                        
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            env.close()

def deploy(SCALE):
    net = torch.load(f"/home/siri/Projects/rewardlearning-robot/behavior_cloning/{DEMO}.pt")
    env = LowDimRobotEnv(ip_address="127.0.0.1", hz=HZ)
    net.eval()
    net.double()
    while True:
            obs = env.reset()
            # preds = torch.load("out_ac.pt")
            print('press "a" then "enter" to start, hit "a" again to stop')
            console = input()
            a = console == 'a'
            d = console == 'd'
            while not a and not d:
                console = input()
                a = console == 'a'
                d = console == 'd'

            if d:
                break

            def isData():
                return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

            old_settings = termios.tcgetattr(sys.stdin)
            updates = []
            try:
                tty.setcbreak(sys.stdin.fileno())
                with torch.no_grad():
                    while True:
                        # Get Button Input (only if True) --> handle extended button press...
                        # Terminate...
                        if isData():
                            c = sys.stdin.read(1)
                            print(c)
                            if c == 'a':  # x1b is ESC
                                plt.clf()
                                fig, ax = plt.subplots(4, 1)
                                updates = np.array(updates)
                                for j in range(4):
                                    ax[j].plot(np.arange(0, len(updates), 1), updates[:, j])
                                plt.savefig('current_traj')
                                updates = []
                                print("\tHit (a) stop deploying\n")
                                break

                        # Otherwise no termination, keep on recording...
                        pos = torch.tensor(obs['lowdim_ee']).double().to(device)
                        update = np.array(net(pos).cpu())``
                        updates.append(update)
                        print(update)
                        obs, _, _, _ = env.step(action=update)
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            env.close()

if __name__ == '__main__':
    main()