import torch
import torch.nn as nn
from torch import optim
import torch.nn.functional as F

import numpy as np
import os
from tqdm import tqdm
import logging

from ballsbot.odometry_fix import image_size, Down, DoubleConv


class CloudDataset(object):
    def __init__(self, root):
        self.root = root
        self.clouds = list(sorted(os.listdir(os.path.join(root, "clouds"))))

    def __getitem__(self, idx):
        cloud_path = os.path.join(self.root, "clouds", self.clouds[idx])
        with open(cloud_path, 'rb') as hf:
            cloud = np.frombuffer(hf.read(), dtype=np.float).reshape((1, image_size, image_size))
        cloud_tensor = torch.as_tensor(np.array(cloud, dtype=np.float), dtype=torch.float32)

        return cloud_tensor, cloud_tensor

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


class UNet(nn.Module):
    def __init__(self, n_channels, bilinear=True):
        super(UNet, self).__init__()
        self.n_channels = n_channels
        self.bilinear = bilinear

        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        factor = 2 if bilinear else 1
        self.down4 = Down(512, 1024 // factor)
        self.up1 = Up(1024, 512 // factor, bilinear)
        self.up2 = Up(512, 256 // factor, bilinear)
        self.up3 = Up(256, 128 // factor, bilinear)
        self.up4 = Up(128, 64, bilinear)
        self.outc = OutConv(64, n_channels)

    def forward(self, x):
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)
        x5 = self.down4(x4)
        x = self.up1(x5, x4)
        x = self.up2(x, x3)
        x = self.up3(x, x2)
        x = self.up4(x, x1)
        logits = self.outc(x)
        return logits


def eval_net(net, loader, device):
    net.eval()
    n_val = len(loader)  # the number of batch
    tot = 0
    loss = nn.MSELoss()
    with tqdm(total=n_val, desc='Validation round', unit='batch', leave=False) as pbar:
        for batch in loader:
            imgs, true_masks = batch
            imgs = imgs.to(device=device, dtype=torch.float32)
            true_masks = true_masks.to(device=device, dtype=torch.float32)

            with torch.no_grad():
                mask_pred = net(imgs)

            tot += loss(mask_pred, true_masks).item()
            pbar.update()

    net.train()
    return tot / n_val


def train_net(
        net,
        device,
        epochs=5,
        batch_size=1,
        lr=0.001,
        save_cp=True,
):
    dataset_train = CloudDataset('/home/jumper/Documents/lidar-clouds/train_unet')
    train_loader = torch.utils.data.DataLoader(
        dataset_train, batch_size=batch_size, shuffle=True, num_workers=1
    )
    n_train = len(dataset_train)

    dataset_val = CloudDataset('/home/jumper/Documents/lidar-clouds/validate_unet')
    val_loader = torch.utils.data.DataLoader(
        dataset_val, batch_size=batch_size, shuffle=False, num_workers=1
    )
    n_val = len(dataset_val)

    # writer = SummaryWriter(comment=f'LR_{lr}_BS_{batch_size}')
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

    optimizer = optim.RMSprop(net.parameters(), lr=lr, weight_decay=1e-8, momentum=0.9)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=2)
    criterion = nn.MSELoss()

    for epoch in range(epochs):
        net.train()

        epoch_loss = 0
        with tqdm(total=n_train, desc=f'Epoch {epoch + 1}/{epochs}', unit='img') as pbar:
            for batch in train_loader:
                imgs = batch[0]
                true_masks = batch[1]
                assert imgs.shape[1] == net.n_channels, \
                    f'Network has been defined with {net.n_channels} input channels, ' \
                    f'but loaded images have {imgs.shape[1]} channels. Please check that ' \
                    'the images are loaded correctly.'

                imgs = imgs.to(device=device, dtype=torch.float32)
                true_masks = true_masks.to(device=device, dtype=torch.float32)

                masks_pred = net(imgs)
                loss = criterion(masks_pred, true_masks)
                epoch_loss += loss.item()
                # writer.add_scalar('Loss/train', loss.item(), global_step)

                pbar.set_postfix(**{'loss (batch)': loss.item()})

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_value_(net.parameters(), 0.1)
                optimizer.step()

                pbar.update(imgs.shape[0])
                global_step += 1
                if global_step % (n_train // (10 * batch_size)) == 0:
                    for tag, value in net.named_parameters():
                        tag = tag.replace('.', '/')
                        # writer.add_histogram('weights/' + tag, value.data.cpu().numpy(), global_step)
                        # writer.add_histogram('grads/' + tag, value.grad.data.cpu().numpy(), global_step)
                    val_score = eval_net(net, val_loader, device)
                    scheduler.step(val_score)
                    # writer.add_scalar('learning_rate', optimizer.param_groups[0]['lr'], global_step)

                    logging.info('Validation MSE: {}'.format(val_score))
                    # writer.add_scalar('Loss/test', val_score, global_step)

                    # writer.add_images('images', imgs, global_step)
                    # if net.n_classes == 1:
                    #     writer.add_images('masks/true', true_masks, global_step)
                    #     writer.add_images('masks/pred', torch.sigmoid(masks_pred) > 0.5, global_step)

        dir_checkpoint = '/home/jumper/Documents/lidar-clouds/checkpoints_unet/'
        if save_cp:
            try:
                os.mkdir(dir_checkpoint)
                logging.info('Created checkpoint directory')
            except OSError:
                pass
            torch.save(net.state_dict(),
                       dir_checkpoint + f'CP_epoch{epoch + 1}.pth')
            logging.info(f'Checkpoint {epoch + 1} saved !')

    # writer.close()


logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = UNet(n_channels=1).to(device=device)

action = 'diff_image'

if action == 'train':
    train_net(
        net=model,
        epochs=10,
        batch_size=4,
        lr=0.005,
        device=device,
    )
elif action == 'show_net':
    from torchsummary import summary

    summary(model, (1, image_size, image_size))
elif action == 'diff_image':
    from PIL import Image

    dataset_train = CloudDataset('/home/jumper/Documents/lidar-clouds/train_unet')
    an_image, _ = dataset_train[len(dataset_train) // 2]
    model.load_state_dict(
        torch.load('/home/jumper/Documents/lidar-clouds/checkpoints_unet/CP_epoch4.pth', map_location=device))
    model.eval()
    restored_image = model(an_image.reshape((1, 1, image_size, image_size)).to(device=device)).cpu()[0]

    Image.fromarray(np.array(
        an_image.reshape((image_size, image_size)).data.numpy() % 256, dtype=np.ubyte
    ), 'L').show()

    Image.fromarray(np.array(
        restored_image.reshape((image_size, image_size)).data.numpy() % 256, dtype=np.ubyte
    ), 'L').show()
