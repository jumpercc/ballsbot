import numpy as np
import torch
import torch.nn as nn

image_size = 128


# some code from https://github.com/milesial/Pytorch-UNet
class DoubleConv(nn.Module):
    """(convolution => [BN] => ReLU) * 2"""

    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        if not mid_channels:
            mid_channels = out_channels
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(mid_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(mid_channels, out_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.double_conv(x)


class Down(nn.Module):
    """Downscaling with maxpool then double conv"""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.maxpool_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_channels, out_channels)
        )

    def forward(self, x):
        return self.maxpool_conv(x)


class UNetEncoder(nn.Module):
    def __init__(self, n_channels, bilinear=True):
        super(UNetEncoder, self).__init__()
        self.n_channels = n_channels
        self.bilinear = bilinear

        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        factor = 2 if bilinear else 1
        self.down4 = Down(512, 1024 // factor)

    def forward(self, x):
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)
        x5 = self.down4(x4)
        return x5


class TwoUNetsEval(nn.Module):
    def __init__(self, n_channels):
        super(TwoUNetsEval, self).__init__()

        uname_encoder = UNetEncoder(n_channels=n_channels)
        self.uname_encoder = uname_encoder
        for param in self.uname_encoder.parameters():
            param.requires_grad = False

        self.tail = nn.Sequential(
            nn.Flatten(),
            nn.Linear(65536, 256),
            nn.Dropout(0.02),
            nn.Linear(256, 256),
            nn.Dropout(0.02),
            nn.Linear(256, 3),
        )
        for param in self.tail.parameters():
            param.requires_grad = False

    def forward(self, x):
        input_shape = list(x.shape)
        split_shape = input_shape.copy()
        split_shape[1] = 1
        features1 = self.uname_encoder(x[:, 0].reshape(tuple(split_shape)))
        features2 = self.uname_encoder(x[:, 1].reshape(tuple(split_shape)))
        two_unets = torch.cat((features1, features2), 1)
        return self.tail(two_unets)


def get_model(from_checkpoint='/home/jumper/Documents/lidar-clouds/checkpoints_two_unets/CP_epoch8.pth'):
    model = TwoUNetsEval(n_channels=1)
    model.load_state_dict(torch.load(from_checkpoint))
    return model


def clouds_pair_to_tensor(points_a, points_b, only_nearby_meters=8):
    results = []
    for points in [points_a, points_b]:
        result = np.full((image_size, image_size), 50., dtype=np.float)
        for x, y in points:
            x_index = int(round(
                (x / only_nearby_meters + 1) * image_size / 2.
            ))
            y_index = int(round(
                (y / only_nearby_meters + 1.) * image_size / 2.
            ))
            if 0 <= x_index < image_size and 0 <= y_index < image_size:
                result[x_index, y_index] = 200.
        results.append(result)
    return torch.as_tensor(
        np.array(results, dtype=np.float), dtype=torch.float32
    ).reshape(
        (1, 2, image_size, image_size)
    )
