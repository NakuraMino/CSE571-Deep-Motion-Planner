from torch.utils.data import Dataset
import torch
import pandas as pd
import numpy as np
import os
import cv2


class HolonomicDataset(Dataset):
    """
    The dataset object for the Holonomic robot data.
    To be used with `torch.utils.data.DataLoader`

    For reference, columns in data.csv are:
    curr_y, curr_x, goal_y, goal_x, mapImage_path, action
    """

    def __init__(self, csv_file, path, grayscale=True):
        """
        :arg csv_file: filename. string ('data.csv')
        :arg path: path to the csv file. string ('../data/image_dataset/')
                   (absolute path)
        """
        self.datafile = csv_file
        self.path = path
        self.data = pd.read_csv(self.path + self.datafile, header=None)
        self.grayscale = grayscale

    def __len__(self):
        """
        Enables using len() on the object

        returns: length of dataset (# rows in csv)
        """
        return len(self.data)

    def __getitem__(self, i):
        """
        Makes the dataset object iterable.

        :arg i: index of the datapoint to be fetched from the dataset object

        returns: a tuple (ipt, opt) where,
        - ipt is a tuple containing 2 elements: (positions, map_image)
            * positions: a 1d torch tensor of length 4
              containing [curr_y, curr_x, goal_y, goal_x]
            * map_image: a 2d torch tensor of size 128 x 128
              containing map img (if grayscale, otherwise it is 128 x 128 x 3)
        - opt is a 1d torch tensor of size 1
          containing the action to be performed (coded 1-8 for each direction)
        """
        N = len(self.data)

        # Check if index is sensible
        if i < N:
            # Input
            positions = torch.tensor(self.data.iloc[i][:4])
            img_path = os.path.join(self.path, self.data.iloc[i][4][2:])
            if self.grayscale is True:
                map_image = torch.from_numpy(cv2.imread(img_path, 0))
            else:
                map_image = torch.from_numpy(cv2.imread(img_path, 1))
            ipt = (positions, map_image)

            # Output
            opt = torch.tensor(self.data.iloc[i][5])

            return (ipt, opt)

        else:
            print('Index to the dataset should be lesser than dataset size.')
            return

        return (ipt, opt)


# This script is to be used just to load the class.
# The below code is just to test if it is working
# (with a random datapoint)
if __name__ == '__main__':
    # Dataset
    csv_file = 'data.csv'
    path = '../holomonic_data/'
    grayscale = True
    full_dataset = HolonomicDataset(csv_file, path, grayscale=grayscale)

    # Test random point
    rdmpt = round(len(full_dataset) * np.random.random())
    print('Datapoint: {}'.format(rdmpt))
    print(full_dataset[rdmpt])
    cv2.imshow('Datapoint: {}'.format(rdmpt),
               full_dataset[rdmpt][0][1].numpy())
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # To be used thus:
    from torch.utils.data import DataLoader
    dataloader = DataLoader(full_dataset, batch_size=100, shuffle=True)
    print(dataloader)
