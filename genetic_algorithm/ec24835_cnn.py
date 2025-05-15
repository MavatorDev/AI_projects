import torchvision
import torchvision.transforms as transforms
import torch
import torch.nn as nn
import torch.optim as optim
import time
from matplotlib import pyplot as plt
import numpy as np

train_set = torchvision.datasets.FashionMNIST(root = ".", train=True,
download=True,
transform=transforms.ToTensor())
validation_set = torchvision.datasets.FashionMNIST(root = ".", train=False,
download=True,
transform=transforms.ToTensor())
train_loader = torch.utils.data.DataLoader(train_set, batch_size=32, shuffle=True)
validation_loader = torch.utils.data.DataLoader(validation_set, batch_size=32, shuffle=False)
# Fix the seed to be able to get the same randomness across runs and
# hence reproducible outcomes
torch.manual_seed(0)

class myCNN(nn.Module):
    def __init__(self,):
        super().__init__()
        self.network = nn.Sequential(
            nn.Conv2d(1,32,(5,5),stride=(1, 1)),
            nn.ReLU(),
            nn.MaxPool2d((2, 2),(2, 2)),
            nn.Conv2d(32,64,(5,5),stride=(1, 1)),
            nn.ReLU(),
            nn.MaxPool2d((2, 2),(2, 2)),
            nn.Flatten(),
            nn.Linear(1024,1024),
            nn.ReLU(),
            nn.Linear(1024,256),
            nn.ReLU(),
            nn.Linear(256,10)
        )
    def forward(self, x):
        return self.network(x)
        
def train_CNN(lr=0.1,epochs=30,criterion=nn.CrossEntropyLoss()):
     # Create the model from the genome
    model = myCNN()
    def init_weights(m):
     if type(m) == nn.Linear:
         torch.nn.init.xavier_uniform_(m.weight,gain=nn.init.calculate_gain('relu'))
    model.apply(init_weights)
    # suggested optimizer to train your models
    optimizer = optim.SGD(model.parameters(), lr=lr)

    # Train the model
    model.train()
    total_loss = 0
    total_batches = len(train_loader)
    val_total_batches = len(validation_loader)
    loss_list=[]
    accuracy_list=[]
    eval_loss_list=[]
    eval_accuracy_list=[]
    speed = []
    for epoch in range(epochs):
        model.train()
        correct = 0
        total = 0
        start_time = time.time()
        epoch_loss = 0
        for batch_idx, (data, target) in enumerate(train_loader):
            optimizer.zero_grad()
            output = model(data)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            pred = output.argmax(dim=1)
            correct += pred.eq(target).sum().item()
            total += target.size(0)


        average_epoch_loss = epoch_loss / val_total_batches
        loss_list.append(average_epoch_loss)
        actual_time=time.time()- start_time
        print("--- %s seconds ---" % (actual_time))
        speed.append(actual_time)
        accuracy_train = correct / total
        accuracy_list.append(accuracy_train)
        print("Accuracy Train: ", accuracy_train)
        total_loss += epoch_loss

         # Evaluate the model
        model.eval()
        correct = 0
        total = 0
        epoch_loss = 0
        for data, target in validation_loader:
            output = model(data)
            loss = criterion(output, target)
            epoch_loss += loss.item()
            pred = output.argmax(dim=1)
            correct += pred.eq(target).sum().item()
            total += target.size(0)

        average_epoch_loss = epoch_loss / val_total_batches
        eval_loss_list.append(average_epoch_loss)
        accuracy_test = correct / total
        eval_accuracy_list.append(accuracy_test)
        print("Evaluation Accuracy: ", accuracy_test)

    torch.save(model.state_dict(), 'best_cnn.pth')
    print(f'Training complete.')
    x = np.linspace(1,epochs,epochs)
    plt.figure()
    plt.plot(x,loss_list,label="Train")
    plt.plot(x,eval_loss_list, label="Validation")
    plt.legend(loc="upper left")
    plt.savefig("loss.PNG")

    plt.figure()
    plt.plot(x,accuracy_list, label="Train")
    plt.plot(x,eval_accuracy_list, label="Validation")
    plt.legend(loc="upper left")
    plt.savefig("accuracy.PNG")

    plt.figure()
    plt.plot(x,speed, label="Speed")
    plt.legend(loc="upper left")
    plt.savefig("speed.PNG")
    
       
    return accuracy_train,correct,total

accuracy,correct,total = train_CNN()