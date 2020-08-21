import torch
import torch.nn as nn
import torch.nn.functional as F
# from torch.utils.tensorboard import SummaryWriter

import numpy as np
import os
from tqdm import tqdm
import logging
import json

from ballsbot.odometry_fix import image_size, Down, DoubleConv


class CloudsPairDataset(object):
    def __init__(self, root):
        self.root = root
        self.clouds = list(sorted(os.listdir(os.path.join(root, "clouds"))))
        self.answers = list(sorted(os.listdir(os.path.join(root, "transformations"))))

    def __getitem__(self, idx):
        cloud_path = os.path.join(self.root, "clouds", self.clouds[idx])
        with open(cloud_path, 'rb') as hf:
            clouds_pair = np.frombuffer(hf.read(), dtype=np.float).reshape((2, image_size, image_size))
        clouds_pair_tensor = torch.as_tensor(np.array(clouds_pair, dtype=np.float), dtype=torch.float32)

        answer_path = os.path.join(self.root, "transformations", self.answers[idx])
        with open(answer_path, 'r') as hf:
            answers = np.array(json.loads(hf.read()))
        answers_tensor = torch.as_tensor(answers, dtype=torch.float32)

        return clouds_pair_tensor, answers_tensor

    def __len__(self):
        return len(self.clouds)


class Up(nn.Module):
    """Upscaling then double conv"""

    def __init__(self, in_channels, out_channels, bilinear=True):
        super().__init__()

        # if bilinear, use the normal convolutions to reduce the number of channels
        if bilinear:
            self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
            self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
        else:
            self.up = nn.ConvTranspose2d(in_channels, in_channels // 2, kernel_size=2, stride=2)
            self.conv = DoubleConv(in_channels, out_channels)

    def forward(self, x1, x2):
        x1 = self.up(x1)
        # input is CHW
        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]

        x1 = F.pad(x1, [diffX // 2, diffX - diffX // 2,
                        diffY // 2, diffY - diffY // 2])
        # if you have padding issues, see
        # https://github.com/HaiyongJiang/U-Net-Pytorch-Unstructured-Buggy/commit/0e854509c2cea854e247a9c615f175f76fbb2e3a
        # https://github.com/xiaopeng-liao/Pytorch-UNet/commit/8ebac70e633bac59fc22bb5195e513d5832fb3bd
        x = torch.cat([x2, x1], dim=1)
        return self.conv(x)


class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(OutConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1)

    def forward(self, x):
        return self.conv(x)


class UNetEncoderFullNet(nn.Module):
    def __init__(self, n_channels, bilinear=True, encoder_only=False):
        super(UNetEncoderFullNet, self).__init__()
        self.n_channels = n_channels
        self.bilinear = bilinear

        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        factor = 2 if bilinear else 1
        self.down4 = Down(512, 1024 // factor)
        if not encoder_only:
            self.up1 = Up(1024, 512 // factor, bilinear)
            self.up2 = Up(512, 256 // factor, bilinear)
            self.up3 = Up(256, 128 // factor, bilinear)
            self.up4 = Up(128, 64, bilinear)
            self.outc = OutConv(64, n_channels)

    def clean_up(self):
        self.up1 = None
        self.up2 = None
        self.up3 = None
        self.up4 = None
        self.outc = None

    def forward(self, x):
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)
        x5 = self.down4(x4)
        # x = self.up1(x5, x4)
        # x = self.up2(x, x3)
        # x = self.up3(x, x2)
        # x = self.up4(x, x1)
        # logits = self.outc(x)
        # return logits
        return x5


class TwoUNets(nn.Module):
    def __init__(self, n_channels, unet_weights_path, unet_encoder_only=False, train_full_net=False):
        super(TwoUNets, self).__init__()
        uname_encoder = UNetEncoderFullNet(
            n_channels=n_channels,
            encoder_only=unet_encoder_only
        )
        if unet_weights_path is not None:
            # load all parameters but keep only encoder
            uname_encoder.load_state_dict(torch.load(unet_weights_path))
            uname_encoder.clean_up()
        self.uname_encoder = uname_encoder

        self.train_full_net = train_full_net
        if not train_full_net:
            # freeze unets
            for param in uname_encoder.parameters():
                param.requires_grad = False

        self.tail = nn.Sequential(
            nn.Flatten(),
            nn.Linear(65536, 256),
            nn.Dropout(0.02),
            nn.Linear(256, 256),
            nn.Dropout(0.02),
            nn.Linear(256, 3),
        )

    def forward(self, x):
        input_shape = list(x.shape)
        split_shape = input_shape.copy()
        split_shape[1] = 1
        features1 = self.uname_encoder(x[:, 0].reshape(tuple(split_shape)))
        features2 = self.uname_encoder(x[:, 1].reshape(tuple(split_shape)))
        two_unets = torch.cat((features1, features2), 1)
        return self.tail(two_unets)

    def eval(self):
        super(TwoUNets, self).eval()
        if train_full_net:
            for_params = self
        else:
            for_params = self.tail
        for param in for_params.parameters():
            param.requires_grad = False

    def train(self, mode=True):
        super(TwoUNets, self).train(mode)
        if train_full_net:
            for_params = self
        else:
            for_params = self.tail
        for param in for_params.parameters():
            param.requires_grad = True


def eval_net(net, loader, device):
    net.eval()
    n_val = len(loader)  # the number of batch
    tot = 0
    loss = nn.SmoothL1Loss()
    with tqdm(total=n_val, desc='Validation round', unit='batch', leave=False) as pbar:
        for batch in loader:
            imgs, transformations = batch
            imgs = imgs.to(device=device, dtype=torch.float32)
            transformations = transformations.to(device=device, dtype=torch.float32)

            with torch.no_grad():
                transformations_pred = net(imgs)

            tot += loss(transformations_pred, transformations).item()
            pbar.update()

    net.train()
    return tot / n_val


def train_net(
        net,
        device,
        epochs=5,
        has_epoch=0,
        batch_size=1,
        lr=0.001,
        save_cp=True,
):
    dataset_train = CloudsPairDataset('/home/jumper/Documents/lidar-clouds/train_two_unets')
    train_loader = torch.utils.data.DataLoader(
        dataset_train, batch_size=batch_size, shuffle=True, num_workers=1
    )
    n_train = len(dataset_train)

    dataset_val = CloudsPairDataset('/home/jumper/Documents/lidar-clouds/validate_two_unets')
    val_loader = torch.utils.data.DataLoader(
        dataset_val, batch_size=batch_size, shuffle=False, num_workers=1
    )
    n_val = len(dataset_val)

    global_step = 0

    logging.info(f'''Starting training:
        Epochs:          {epochs}
        Batch size:      {batch_size}
        Learning rate:   {lr}
        Training size:   {n_train}
        Validation size: {n_val}
        Checkpoints:     {save_cp}
        Device:          {device.type}
    ''')

    # optimizer = optim.RMSprop(net.parameters(), lr=lr, weight_decay=1e-8, momentum=0.9)
    # scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=2)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.SmoothL1Loss()

    for epoch in range(has_epoch, epochs):
        epoch_loss = 0
        with tqdm(total=n_train, desc=f'Epoch {epoch + 1}/{epochs}', unit='img') as pbar:
            for batch in train_loader:
                imgs = batch[0]
                # logging.info(imgs.shape)
                transformations = batch[1]
                # logging.info(transformations.shape)
                # logging.info(transformations)

                imgs = imgs.to(device=device, dtype=torch.float32)
                transformations = transformations.to(device=device, dtype=torch.float32)

                transformations_pred = net(imgs)
                # logging.info(transformations_pred.shape)
                # logging.info(transformations_pred)
                loss = criterion(transformations_pred, transformations)
                epoch_loss += loss.item()

                pbar.set_postfix(**{'loss (batch)': loss.item()})

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_value_(net.parameters(), 0.1)
                optimizer.step()

                pbar.update(imgs.shape[0])
                global_step += 1
                if global_step % (n_train // (10 * batch_size)) == 0:
                    val_score = eval_net(net, val_loader, device)
                    # scheduler.step(val_score)
                    logging.info('Validation loss: {}'.format(val_score))

        dir_checkpoint = '/home/jumper/Documents/lidar-clouds/checkpoints_two_unets/'
        if save_cp:
            try:
                os.mkdir(dir_checkpoint)
                logging.info('Created checkpoint directory')
            except OSError:
                pass
            torch.save(net.state_dict(),
                       dir_checkpoint + f'CP_epoch{epoch + 1}.pth')
            logging.info(f'Checkpoint {epoch + 1} saved !')


action = 'train'
has_epoch = 8
train_full_net = True

if has_epoch > 0:
    unet_checkpoint = None
else:
    unet_checkpoint = '/home/jumper/Documents/lidar-clouds/checkpoints_unet/CP_epoch4.pth'

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = TwoUNets(
    n_channels=1,
    unet_weights_path=unet_checkpoint,
    unet_encoder_only=has_epoch > 0,
    train_full_net=train_full_net,
)
if has_epoch > 0:
    model.load_state_dict(torch.load(
        '/home/jumper/Documents/lidar-clouds/checkpoints_two_unets/CP_epoch{}.pth'.format(has_epoch)
    ))
    model.uname_encoder.clean_up()
model.to(device=device)

if action == 'train':
    train_net(
        net=model,
        epochs=10,
        has_epoch=has_epoch,
        batch_size=4,
        lr=0.005,
        device=device,
    )
elif action == 'show_net':
    from torchsummary import summary

    summary(model, (2, image_size, image_size))
