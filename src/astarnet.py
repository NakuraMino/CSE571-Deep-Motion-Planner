# -*- coding: utf-8 -*-
"""AStarNet.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1_W4xQvnQFL1mCjuIEtcoCNpLS3anniN7
"""

# !unzip images.zip

import numpy as np
import torch 
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from dataloader import HolonomicDataset
from torch.utils.data import DataLoader

class AStarNet(nn.Module):
  def __init__(self):
    super(AStarNet, self).__init__()
    self.conv1 = nn.Conv2d(1, 8, 3, padding=1)
    self.conv2 = nn.Conv2d(8, 16, 3, padding=1)
    self.conv3 = nn.Conv2d(16, 32, 3, padding=1)
    self.conv4 = nn.Conv2d(32, 64, 3, padding=1)
    self.fc1 = nn.Linear(8 * 8 * 64 + 4, 256)
    self.fc2 = nn.Linear(256, 128)
    self.fc3 = nn.Linear(128, 64)
    self.fc4 = nn.Linear(64, 8)

  def forward(self, x):
    config_states, maps = x
    maps = maps.unsqueeze(1).float() # .to("cuda")
    config_states = config_states.float()# .to("cuda")
    x = F.max_pool2d(F.relu(self.conv1(maps)), 2)
    x = F.max_pool2d(F.relu(self.conv2(x)), 2)
    x = F.max_pool2d(F.relu(self.conv3(x)), 2)
    x = F.max_pool2d(F.relu(self.conv4(x)), 2)
    x = x.view(-1, self.num_flat_features(x))
    x = torch.cat((x, config_states), axis=1)
    x = F.relu(self.fc1(x))
    x = F.relu(self.fc2(x))
    x = F.relu(self.fc3(x))
    x = self.fc4(x)
    return x
  
  def num_flat_features(self, x):
    size = x.size()[1:]  # all dimensions except the batch dimension
    num_features = 1
    for s in size:
        num_features *= s
    return num_features

if __name__ == "__main__":
      
  path = './'
  dataset = HolonomicDataset('data.csv', path, grayscale=True)
  dataloader = DataLoader(dataset, batch_size=128, shuffle=True)

  net = AStarNet().to("cuda")
  optimizer = optim.Adam(net.parameters(), lr=0.001)
  criterion = nn.CrossEntropyLoss()
  JHist = []

  # overfit on one batch to make sure everything is working
  for i in range(5000):
    optimizer.zero_grad()
    y_pred = net(data)
    loss = criterion(y_pred, label)
    loss.backward()
    optimizer.step()

    epoch_loss = loss.item()
    JHist.append(epoch_loss)
    print(i, epoch_loss)

  for j in range(120):
    for i, (input_data, labels) in enumerate(dataloader):

      y_label = labels.to("cuda")
      y_label = y_label - 1
      optimizer.zero_grad()
      y_pred = net(input_data)
      loss = criterion(y_pred, y_label)
      loss.backward()
      optimizer.step()

      epoch_loss = loss.item()
      JHist.append(epoch_loss)
      
      print(i + j * 83, epoch_loss)

  import matplotlib.pyplot as plt
  print(JHist)
  plt.plot(JHist)

  JHist_avg = []
  sum = 0
  for index in range(30, len(JHist)):
    sum += JHist[index]
    if index % 9 == 0:
      JHist_avg.append(sum / 10)
      sum = 0

  plt.plot(JHist_avg)

  # check labels on one batch
  test_input, test_label = None, None
  for i, (input_data, labels) in enumerate(dataloader):
    test_input = input_data
    test_label = labels.to("cuda")
    break

  y_pred = net(test_input)
  print(y_pred.shape)
  pred_actions = torch.argmax(y_pred, axis=1)
  print(pred_actions)
  print(test_label - 1)

  correct = 0
  for i in range(128):
    if pred_actions[i] == test_label[i] - 1:
      correct += 1
  print(correct / 128)

  from google.colab import files
  PATH = './AStarNet.pth'
  torch.save(net.state_dict(), PATH)
  files.download(PATH)
