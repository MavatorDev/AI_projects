import os
import random
import numpy as np
import librosa
import scipy.io
from scipy.signal.windows import hann
from sklearn.model_selection import train_test_split


import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset


# We suggest your GA uses at least these hyper-parameters
# The values stated here are suboptimal and you should tune them
# You should also consider adding more hyper-parameters
population_size = 7
num_generations = 4
num_parents = 2



# these are other global hyper-parameters that apply across all neural networks.
# You could change these if you want, but they should NOT
# be the focus of your attention in this assignment.
# Note that if you do not change them at all, your algorithm
# will still likely be optimal.
BATCH_SIZE = 8
LR = 0.01
EPOCHS = 5
activation_functions = [
        nn.ReLU(),          
        nn.LeakyReLU(),     
        nn.PReLU(),         
        nn.ELU(),           
        nn.SELU(),          
        nn.Sigmoid(),       
        nn.Tanh(),          
        nn.Softmax(dim=1),  
        nn.Softplus(),      
        nn.Hardtanh(),      
        nn.GELU(),          
        nn.SiLU()]

text_act = [str(activation_functions[i]).lower().split('(')[0] for i in range(len(activation_functions))]

# You do not have to modify this function, but you could, for example, do data augmentation,
# which would aid your ability to find the most optimal model. But don't worry, you will get
# full credit even if you do not modify this function at all.
def load_and_process_pcvc_data(directory='.', train_size=0.8, random_seed=42):
    """
    Load, process, and split the PCVC dataset from .mat files into training and validation sets,
    with randomized window slicing for the training data to expand the dataset size.

    Instructions for data preparation:
    1. Visit the Kaggle dataset page at https://www.kaggle.com/sabermalek/pcvcspeech
    2. Download the dataset by clicking on the 'Download' button.
    3. Extract the downloaded zip file.
    4. Ensure all .mat files from the dataset are in the same directory as this script or specify the directory.

    Parameters:
    - directory: str, the directory where .mat files are located (default is the current directory).
    - train_size: float, the proportion of the dataset to include in the train split.
    - random_seed: int, the seed used for random operations to ensure reproducibility.

    Returns:
    - tr_data: np.array, training dataset.
    - tr_labels: np.array, training labels.
    - vl_data: np.array, validation dataset.
    - vl_labels: np.array, validation labels.
    """

    # List all .mat files in the specified directory
    all_mats = [file for file in os.listdir(directory) if file.endswith('.mat')]
    raw_data = []
    num_vowels = 6
    ndatapoints_per_vowel = 299
    labels = []

    for idx, mat_file in enumerate(all_mats):
        mat_path = os.path.join(directory, mat_file)
        mat_data = np.squeeze(scipy.io.loadmat(mat_path)['x'])
        raw_data.append(mat_data)
        labels.append(np.repeat(np.arange(num_vowels)[np.newaxis], mat_data.shape[0], axis=0))

    # Concatenate and reshape all data
    raw_data, labels = np.concatenate(raw_data, axis=1), np.concatenate(labels, axis=1)
    nreps, nvow, nsamps = raw_data.shape
    raw_data = np.reshape(raw_data, (nreps * nvow, nsamps), order='F')
    labels = np.reshape(labels, (nreps * nvow), order = 'F')

    # Split data into training and validation sets
    tr_data, vl_data, tr_labels, vl_labels = train_test_split(
        raw_data, labels, train_size=train_size, random_state=random_seed, stratify=labels)
    
    # Define window size and function
    window_size = 10000
    window = hann(window_size)

    # Process Training Data with random slicing
    tr_data_processed = []
    tr_labels_processed = []
    for j in range(10): # repeat the tr data 10 times
      for i, d in enumerate(tr_data):
          start = np.random.randint(0, nsamps - window_size)
          end = start + window_size
          sliced = d[start:end] * window
          resampled = librosa.resample(sliced, orig_sr=48000, target_sr=16000)
          tr_data_processed.append(resampled)
          tr_labels_processed.append(tr_labels[i])
    tr_data = np.array(tr_data_processed)
    tr_labels= np.array(tr_labels_processed)

    # Process Validation Data with fixed slicing
    vl_data = vl_data[:, 5000:15000] * window
    vl_data = np.array([librosa.resample(d, orig_sr=48000, target_sr=16000) for d in vl_data])

    # One-hot encode labels
    tr_labels = np.eye(num_vowels)[tr_labels]
    vl_labels = np.eye(num_vowels)[vl_labels]

    return tr_data, tr_labels.astype('float'), vl_data, vl_labels.astype('float')

class Net(nn.Module):
    """
    Defines a neural network architecture dynamically based on a specified genome configuration.

    The `Net` class constructs a neural network where each layer's configuration is dictated by the genome.
    The network will always end with a linear layer with an output size of `K`, meant
    to match the number of classes in the dataset.

    TIP: using the nn.Sequential API will simplify your code to define models using the suggested genome format.

    Parameters:
    - genome (list of dicts): Specifies the architecture of the neural network. Each dictionary in the list
      represents a layer in the network and should include keys for 'num_neurons' (int), 'activation' (str),
      and optionally 'dropout_rate' (float).
    - D (int): The dimensionality of the input data. Defaults to 3.
    - K (int): The number of output classes. Defaults to 4.

    Attributes:
    - network (nn.Sequential): The sequential container of network layers as specified by the genome.
    """
    def __init__(self, genome, D=3, K=4):
        super().__init__()
        act = activation_functions[0]
        layers = []
        input_features = D
        for i in range(len(genome)+1):
            if i == 0:
                layers.append(nn.Linear(D,genome[0]['num_neurons']))
                act_f = activation_functions[text_act.index(genome[i]['activation'])]
                layers.append(act)
                
            if i <= len(genome)-1 and i>0:
                layers.append(nn.Linear( genome[i-1]['num_neurons'],genome[i]['num_neurons']))
                act_f = activation_functions[text_act.index(genome[i]['activation'])]
                layers.append(act)
                if genome[i]['dropout']:
                    layers.append(nn.Dropout(genome[i]['dropout']))

            if i==len(genome):
                layers.append(nn.Linear( genome[i-1]['num_neurons'],K))
        
        self.network = nn.Sequential(*layers)

    # you shouldn't have to modify this method
    def forward(self, x):
        """
        Defines the forward pass of the neural network.

        Parameters:
        - x (Tensor): The input data tensor.

        Returns:
        - Tensor: The output of the network after processing the input tensor through all the layers defined
          in the `network` attribute.
        """
        return self.network(x)


def generate_initial_population(size, blueprint):
    """
    Generates an initial population of neural network architectures based on a flexible blueprint.

    Each individual in the population (or 'genome') consists of a randomly constructed neural network architecture.
    The architecture is determined by randomly selecting from possible configurations specified in the blueprint.

    Parameters:
    - size (int): The number of neural network architectures to generate in the population.
    - blueprint (dict): A dictionary specifying the possible configurations for neural network layers.
      The blueprint can contain keys such as:
      - 'n_layers' (int): The number of layers to include in each network architecture.
      - 'neurons' (list): Possible numbers of neurons per layer.
      - 'activations' (list): Possible activation functions.
      - 'dropout' (list): Possible dropout rates, including None if dropout is not to be applied.
      - other parameters as you see fit.

      Each layer in a generated architecture randomly selects from these lists, promoting a diverse initial population.

    Returns:
    - population (list of list of dicts): A list of neural network architectures, where each architecture
      is represented as a list of dictionaries. Each dictionary defines the configuration of one layer.

    Example:
    >>> population = generate_initial_population(10, blueprint)
    >>> len(population)
    10
    """
    population = []
    for  i in range(size):
        element=[]
        max_layers_rnd = np.random.randint(1,blueprint['max_n_layers'])
        for j in range(max_layers_rnd):
            neuron_i=np.random.randint(len(blueprint['neurons']))
            activation_i=np.random.randint(len(blueprint['activations']))
            dropout_i=blueprint['dropout'][np.random.randint(len(blueprint['dropout']))]
        
            element.append({
            "num_neurons": blueprint['neurons'][neuron_i],
            "activation":  blueprint['activations'][activation_i],
            "dropout": dropout_i})
        population.append(element)
    return population


def selection(population, fitnesses, num_parents):
    """
    Selects the top-performing individuals from the population based on their fitness scores.

    This function sorts the population by fitness in descending order and selects the top `num_parents`
    individuals to form the next generation's parent group. This selection process ensures that individuals
    with higher fitness have a higher probability of reproducing and passing on their genes.

    Parameters:
    - population (list of list of dicts): The population from which to select top individuals. Each individual
      in the population is represented as a genome, which is a list of dictionaries where each dictionary
      details the configuration of a neural network layer.
    - fitnesses (list of floats): A list of fitness scores corresponding to each individual in the population.
      Each fitness score should be a float indicating the performance of the associated individual.
    - num_parents (int): The number of top-performing individuals to select for the next generation.

    Returns:
    - list of list of dicts: A list containing the genomes of the top-performing individuals selected from the
      population.

    Example:
    >>> population = [[{'num_neurons': 32, 'activation': 'relu'}], [{'num_neurons': 16, 'activation': 'sigmoid'}]]
    >>> fitnesses = [0.95, 0.88]
    >>> selected = selection(population, fitnesses, 1)
    >>> len(selected)
    1
    """
    fitnessesc=fitnesses.copy()
    populationc=population.copy()
    sorted_population=[]
    for i in range(num_parents):
        max_score=fitnessesc.index(max(fitnessesc))
        fitnessesc.pop(max_score)
        sorted_population.append(populationc.pop(max_score))
    return sorted_population

# feel free to add keyword arguments to this function as you see fit
def crossover(parent1, parent2):
    """
    Combines two parent genomes to create a new child genome through a crossover process of your choice.

    Parameters:
    - parent1 (list of dicts): The genome of the first parent.
    - parent2 (list of dicts): The genome of the second parent.
    - other keyword arguments as you may need

    Returns:
    - list of dicts: The genome of the child, formed by combining genes from the two parents.
    """
    
    child=[]
    mean_length = int(np.mean([len(parent1),len(parent2)]))
    for i in range(mean_length):
        if i>len(parent2)-1:
            parent2 = parent1
        if i>len(parent1)-1:
            parent1 = parent2    
        parents_activation = [parent1[i]['activation'],parent2[i]['activation']]
        activation_child=parents_activation[np.random.randint(len(parents_activation))]
        parents_dropout = [parent1[i]['dropout'],parent2[i]['dropout']]
        dropout_child = parents_dropout[np.random.randint(len(parents_dropout))]
        child.append({
        'num_neurons':int(np.mean([parent1[i]['num_neurons'],parent2[i]['num_neurons']])),
		'activation':activation_child,
        'dropout':dropout_child
        })

    return child

# feel free to add keyword arguments to this function as you see fit
def mutate(genome,sigma=5,sigma_drop=0.1,lr=0.1):
    """
    Introduces random changes to a genome based on a specified mutation approach.

    Parameters:
    - genome (list of dicts): The genome to be mutated.
    - other keyword arguments as you may need

    Returns:
    - list of dicts: The mutated genome.
    """    
    
    mutated = []
    leng=len(genome)

    
    for i in range(leng):
        genome_i = genome[i]
        #Calculates the sigma prime
        sigma_p = sigma*np.exp(lr*np.random.normal(0,1,sigma))[0]
        #Updates the number of neurones
        new_neurons=genome_i['num_neurons']
        new_neurons = abs(int(genome_i['num_neurons'] + (sigma_p*np.random.normal(0,1,(1))[0])))

        new_act = genome_i['activation']
        #Randomly chooses if the activation function is gonna be updated
        if np.random.randint(0,2):
            new_act = text_act[np.random.randint(0,len(activation_functions))]

        
        #Dropout is updated by using the normal distribution
        sigma_p_drop = sigma_drop*np.exp(lr*np.random.normal(0,1,(1))[0])
        
        new_drop = sigma_p_drop*np.random.normal(0,1,(1))[0]
        if genome_i['dropout'] :
            if genome_i['dropout']+new_drop <=1 and genome_i['dropout']+new_drop >=0:
                new_drop = genome_i['dropout']+new_drop
            else:
                new_drop = 0.5
        else:
            new_drop = None
        #Updates the new configuration
        mutated.append(
            {
               'num_neurons':new_neurons,
		        'activation':new_act,
                'dropout':new_drop
            }
        )
    return mutated


# You do not have to modify this function
def compute_fitness(genome, train_loader, test_loader, criterion, lr=0.01, epochs=5, D=None, K=None):

    # Create the model from the genome
    model = Net(genome, D, K)

    # suggested optimizer to train your models
    optimizer = optim.Adam(model.parameters(), lr=lr)

    # Train the model
    model.train()
    total_loss = 0
    total_batches = len(train_loader)
    for epoch in range(epochs):
        epoch_loss = 0
        for batch_idx, (data, target) in enumerate(train_loader):
            optimizer.zero_grad()
            output = model(data)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()

        average_epoch_loss = epoch_loss / total_batches
        print(f'Epoch {epoch + 1}/{epochs} complete. Average Training Loss: {average_epoch_loss:.4f}')
        total_loss += epoch_loss

    print(f'Training complete.')

    # Evaluate the model
    model.eval()
    correct = 0
    total = 0
    with torch.no_grad():
        for data, target in test_loader:
            output = model(data)
            pred = output.argmax(dim=1, keepdim=True)
            target = target.argmax(dim=1, keepdim=True)
            correct += pred.eq(target).sum().item()
            total += target.size(0)

    accuracy = correct / total
    print(f'Evaluation complete. Accuracy: {accuracy:.4f} ({correct}/{total} correct)\n')

    return accuracy


########################################
# You don't need to change these lines #
########################################

# Load and process the PCVC dataset into training and validation sets.
# This function will split the raw data, apply preprocessing like windowing and resampling,
# and return processed training data (X, labels) and validation data (X_val, labels_val).
X, labels, X_val, labels_val = load_and_process_pcvc_data()
# Determine the number of samples and classes from the shape of the training data and labels.
# This will help in setting up the network architecture later.
Nsamps, Nclasses = X.shape[-1], labels.shape[-1]
# Convert numpy arrays to PyTorch tensors. Tensors are a type of data structure used in PyTorch
# that are similar to arrays.
X_tensor, X_val_tensor = torch.FloatTensor(X), torch.FloatTensor(X_val)
y_tensor, y_val_tensor = torch.FloatTensor(labels), torch.FloatTensor(labels_val)
# Wrap tensors in a TensorDataset, which provides a way to access slices of tensors
# using indexing that is useful during training because it abstracts away the data handling.
dataset = TensorDataset(X_tensor, y_tensor)
dataset_val = TensorDataset(X_val_tensor, y_val_tensor)
# DataLoader is used to efficiently load data in batches, which is necessary for training neural networks.
# `shuffle=True` ensures that the data is shuffled at every epoch to prevent the model from learning
# any order-based biases in the dataset.
train_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(dataset_val, batch_size=BATCH_SIZE, shuffle=False)


# **************************************** #
# **************************************** #
# You shold modify the following statement #
# **************************************** #
# **************************************** #
# Define your initial population (you will modify and expand this blueprint)
blueprint = {
    'max_n_layers': 3,           # Define the (maximum) number of layers in each neural network
    'neurons': [i for i in range(1,300,5)],              # Possible neuron counts per layer
    'activations': ['sigmoid','relu','softmax','elu'],  # Possible activation functions
    'dropout': [None,0.1,0.2]     
    }

#########################################################################
# Other than passing the keyword arguments you need for your functions, #
# you shouldn't need to change the following lines and their logic,     #
#########################################################################

population = generate_initial_population(population_size, blueprint)
# Initialize best performance tracking
best_overall_fitness = float('-inf')
best_overall_architecture = None

for generation in range(num_generations):

    # Evaluate fitnesses
    fitnesses = []
    total_genomes = len(population)
    for idx, genome in enumerate(population):
        # Compute the fitness for each genome
        fitness = compute_fitness(genome, train_loader, val_loader, nn.CrossEntropyLoss(), lr=LR, epochs=EPOCHS, D=Nsamps, K=Nclasses)
        fitnesses.append(fitness)
        print(f'    Genome {idx + 1}/{total_genomes} evaluated. "Fitness" (i.e. accuracy): {fitness:.4f}.\n')
    print(f"All genomes in generation {generation} have been evaluated.")

    parents = selection(population, fitnesses, num_parents)

    # Track the best architecture in this generation
    max_fitness_idx = fitnesses.index(max(fitnesses))
    print(fitnesses)
    best_fitness_this_gen = fitnesses[max_fitness_idx]
    best_architecture_this_gen = population[max_fitness_idx]

    # Update overall best if the current gen has a new best
    if best_fitness_this_gen > best_overall_fitness:
        best_overall_fitness = best_fitness_this_gen
        best_overall_architecture = best_architecture_this_gen

    print(f"Generation {generation + 1}, Best Fitness: {best_fitness_this_gen}")
    print("Best Architecture:", best_architecture_this_gen, '\n')

    # Generate next generation
    next_generation = parents[:]
    while len(next_generation) < population_size:
        parent1, parent2 = random.sample(parents, 2)
        child = crossover(parent1, parent2)
        child = mutate(child)
        next_generation.append(child)
    population = next_generation

# You do not need to change anything here

# Final summary at the end of all generations
print("\nFinal Summary")
print("Best Overall Fitness:", best_overall_fitness)
print("Best Overall Architecture:", best_overall_architecture)

# Inform about the beginning of the re-training process
print("\nStarting the re-training of the best model found by the genetic algorithm (corroborate reproducibility)")

# Re-build the best model based on the architecture determined to be most effective during the genetic algorithm.
# This model is built from scratch using the best configuration parameters (genome) found.
best_model = Net(best_overall_architecture, D=Nsamps, K=Nclasses)

# Set up the loss function and the optimizer. The optimizer is configured to optimize the weights of our neural network,
# and the learning rate is set as per earlier specification.
best_model_criterion = nn.CrossEntropyLoss()
best_model_optimizer = optim.Adam(best_model.parameters(), lr=LR)

# Training loop: This process involves multiple epochs where each epoch goes through the entire training dataset.
for epoch in range(EPOCHS):
    best_model.train()  # Set the model to training mode
    total_loss = 0
    total_batches = len(train_loader)

    # Process each batch of data
    for batch_idx, (data, target) in enumerate(train_loader):
        best_model_optimizer.zero_grad()  # Clear previous gradients
        output = best_model(data)  # Compute the model's output
        loss = best_model_criterion(output, target)  # Calculate loss
        loss.backward()  # Compute gradients
        best_model_optimizer.step()  # Update weights
        total_loss += loss.item()  # Accumulate the loss

    average_epoch_loss = total_loss / total_batches
    print(f'Epoch {epoch + 1}/{EPOCHS} complete. Average Training Loss: {average_epoch_loss:.4f}')

# After training, switch to evaluation mode for testing.
best_model.eval()
correct = 0
total = 0

# Disable gradient computation for validation, as it isn't needed and saves memory and computation.
with torch.no_grad():
    # Process each batch from the validation set
    for data, target in val_loader:
        output = best_model(data)  # Compute the model's output
        pred = output.argmax(dim=1, keepdim=True)  # Find the predicted class
        target = target.argmax(dim=1, keepdim=True)  # Actual class
        correct += pred.eq(target).sum().item()  # Count correct predictions
        total += target.size(0)  # Total number of items

validation_accuracy = correct / total  # Calculate accuracy
print(f'Evaluation on validation set complete. Accuracy: {validation_accuracy:.4f} ({correct}/{total} correct)')

# Save the trained model's weights for future use.
torch.save(best_model.state_dict(), 'best_net.pth')
print("Saved the best model's weights to 'best_net.pth'")